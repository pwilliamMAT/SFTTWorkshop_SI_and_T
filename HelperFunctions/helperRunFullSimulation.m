function results = helperRunFullSimulation(tuningData, mapOrigin)
%RUNFULLSIMULATION  Phase 2 — full-duration simulation with MEX fuser
%   results = runFullSimulation(tuningData, mapOrigin)
%
%   Re-creates the scenario, tracker, sensors, and fuser from scratch,
%   then runs the complete simulation using the MEX-compiled
%   fusionAlgorithm_mex (which has its own hardcoded correct config).
%
%   Inputs:
%     tuningData  — cell array of truth trajectory structs (from TrackingScenarioTruth.mat)
%     mapOrigin   — [lat lon alt] reference origin (deg, deg, m)
%
%   Output:
%     results     — struct with all logged data and metrics:
%       .mapViewer, .wallClockTime,
%       .truePoseGPSAll, .truePoseNEDAll,
%       .fusedTrackLog, .radarTrackLog, .adsbTrackLog, .adsbTrackLogNED,
%       .posRMSE, .velRMSE, .posANEES, .velANEES,
%       .trackError, .truthError,
%       .trackAssignmentSummary, .truthAssignmentSummary

a = tic;

%% ---- Scenario ----
scenario = createScenario(tuningData, mapOrigin);

%% ---- Globe Viewer ----
mapViewer = trackingGlobeViewer('ReferenceLocation',mapOrigin,'Basemap','streets-dark');
campos(mapViewer, mapOrigin + [0 -0.4 1e5]);
drawnow;
plotScenario(mapViewer, scenario);

%% ---- Sensors ----
horAccuracy  = 20;
vertAccuracy = 50;
velAccuracy  = 0.4;

gps = gpsSensor('PositionInputFormat','Geodetic',...
    'HorizontalPositionAccuracy',horAccuracy,...
    'VerticalPositionAccuracy',vertAccuracy,...
    'VelocityAccuracy',velAccuracy);
numTargets = numel(scenario.Platforms) - 2;

transponderStruct = createTX(numTargets, gps);
adsbRx = adsbReceiver('ReceiverIndex',2);

%% ---- Tracker ----
[tracker, activeRadarSpec, targetSpec] = helperCreateTracker(scenario); %#ok<ASGLU>

%% ---- Load sensor data ----
load('SensorData.mat','sensorData');

% Reset scenario, viewer, transponders
clear(mapViewer);
restart(scenario);
release(tracker);

trackerUpdateInterval = 1;
activeRadarData = splitSensorData(sensorData, scenario.StopTime, trackerUpdateInterval);

%% ---- Preallocate ----
numStepsToRun = numel(activeRadarData);

truePoseGPSAll = cell(1, numStepsToRun);
truePoseNEDAll = cell(1, numStepsToRun);
fusedTrackLog  = cell(1, numStepsToRun);
radarTrackLog  = cell(1, numStepsToRun);
adsbTrackLog   = cell(1, numStepsToRun);
adsbTrackLogNED = cell(1, numStepsToRun);
trackError     = cell(1, numStepsToRun);
truthError     = cell(1, numStepsToRun);

posRMSE  = zeros(1, numStepsToRun);
velRMSE  = zeros(1, numStepsToRun);
posANEES = zeros(1, numStepsToRun);
velANEES = zeros(1, numStepsToRun);

%% ---- Reset everything for sim loop ----
clear(mapViewer);
restart(scenario);
reset(adsbRx);
for i = 1:numTargets
    reset(transponderStruct(i).adsbTx);
end
plotPlatform(mapViewer,[scenario.Platforms{3:end}],TrajectoryMode="Full");

% Labels & colors
adsblabel = "       ADS-B";
radarlabel = "  Radar";
fusedlabel = string(sprintf('%s\n',"","Fused"));
adsbclr  = [183 70 255]/255;
radarclr = [255 255 17]/255;
fusedclr = [255 105 41]/255;

%% ---- Metrics ----
errorMetrics = trackErrorMetrics;
assignmentMetrics = trackAssignmentMetrics(...
    'AssignmentDistance','posabserr',...
    'DivergenceDistance','posabserr',...
    'DivergenceThreshold',50,...
    'AssignmentThreshold',50);

%% ---- Clear MEX to reset persistent fuser state ----
clear fusionAlgorithm_mex fusionAlgorithm

wgs84 = wgs84Ellipsoid('meter');

% Plot every Nth step to reduce globe-viewer overhead
plotInterval = 5;

%% ---- Simulation Loop ----
fprintf('  Running %d steps ...\n', numStepsToRun);

for ind = 1:numStepsToRun
    advance(scenario);

    plotThisStep = (mod(ind, plotInterval) == 0) || ind == 1 || ind == numStepsToRun;

    % Coverage
    covcon = coverageConfig(scenario);
    if plotThisStep
        plotCoverage(mapViewer, covcon, "ECEF", Color=[0 1 0])
    end

    % Truth positions (LLA + NED)
    for i = 1:numel(scenario.Platforms)
        poseStruct = pose(scenario.Platforms{i},'CoordinateSystem','Geodetic');
        poseStruct.PlatformID = scenario.Platforms{i}.PlatformID;
        truePoseGPS(i) = poseStruct;

        tmp = pose(scenario.Platforms{i},'CoordinateSystem','Cartesian');
        tmp.PlatformID = scenario.Platforms{i}.PlatformID;

        pECEF = tmp.Position;
        vECEF = tmp.Velocity;
        [xN, yE, zD] = ecef2ned(pECEF(1),pECEF(2),pECEF(3), mapOrigin(1),mapOrigin(2),mapOrigin(3), wgs84);
        truePoseNED(i).Position = [xN, yE, zD];
        [uN, vE, wD] = ecef2nedv(vECEF(1),vECEF(2),vECEF(3), mapOrigin(1),mapOrigin(2));
        truePoseNED(i).Velocity  = [uN vE wD];
        truePoseNED(i).PlatformID = tmp.PlatformID;
        truePoseNED(i).Orientation = tmp.Orientation;
        truePoseNED(i).AngularVelocity = tmp.AngularVelocity;
    end

    truePoseGPSAll{ind} = truePoseGPS;
    truePoseNEDAll{ind} = truePoseNED;
    truePosePlot = platformPoses(scenario);

    if plotThisStep
        plotPlatform(mapViewer, truePosePlot, 'ECEF');
        plotActiveRadarData(mapViewer, activeRadarSpec, activeRadarData(ind));
    end

    time = (ind - 1) * trackerUpdateInterval;

    % Init track arrays
    adsbTracks     = objectTrack.empty;
    adsbTracksPlot = objectTrack.empty;
    radarTracks    = objectTrack.empty;
    fusedTracks    = objectTrack.empty;

    % --- ADS-B ---
    for i = 3:numel(truePoseGPS)
        position = truePoseGPS(i).Position;
        velocity = truePoseGPS(i).Velocity;
        if ~isnan(position)
            adsbMessages(i-2,1) = transponderStruct(i-2).adsbTx(position,velocity);
        end
    end
    adsbTracks = adsbRx(adsbMessages, time);
    adsbTracks = adsbTracks';
    clear adsbMessages
    adsbTracks = ensureTimeAndParams(adsbTracks, time, ecefParams());

    for i = 1:length(adsbTracks)
        adsbTracksPlot(i) = central2radar(adsbTracks(i), mapOrigin);
    end

    % --- Radar ---
    radarTracksStruct = trackingAlgorithm_mex({activeRadarData(ind)}, targetSpec, activeRadarSpec);
    radarTracks = convertToObjectTrack(radarTracksStruct);
    radarTracks = ensureTimeAndParams(radarTracks, time, nedParams());

    % --- Fusion (MEX) ---
    if ~isempty([adsbTracks;radarTracks])
        trackObj    = [adsbTracks;radarTracks];
        trackStruct = toStructForMEX(trackObj);
        fusedStructs = fusionAlgorithm_mex(trackStruct, time);
        if ~isempty(fusedStructs)
            fusedTracks = convertToObjectTrack(fusedStructs);
        end
    end

    % --- Plot ---
    if plotThisStep
        mapViewer = helperPlotTracks(3,mapViewer,adsbTracksPlot,radarTracks,fusedTracks,...
            adsblabel,adsbclr,radarlabel,radarclr,fusedlabel,fusedclr);
    end

    % --- Log (convert fused ECEF → NED) ---
    fusedTracksNED = objectTrack.empty;
    for i = 1:length(fusedTracks)
        fusedTracksNED(i) = central2radar(fusedTracks(i), mapOrigin);
    end

    fusedTrackLog{ind}  = fusedTracksNED;
    radarTrackLog{ind}  = radarTracks;
    adsbTrackLog{ind}   = adsbTracks;
    adsbTrackLogNED{ind} = adsbTracksPlot;

    % --- Metrics ---
    truthsForMetrics = truePoseNED;
    if numel(truthsForMetrics) > 2
        truthsForMetrics = truthsForMetrics(3:end);
    else
        truthsForMetrics = truthsForMetrics([]);
    end

    [trackAssignmentSummary(ind), truthAssignmentSummary(ind)] = assignmentMetrics(fusedTracksNED, truthsForMetrics);
    [assignedTrackIDs, assignedTruthIDs] = currentAssignment(assignmentMetrics);
    [posRMSE(ind), velRMSE(ind), posANEES(ind), velANEES(ind)] = errorMetrics(fusedTracksNED, assignedTrackIDs, truthsForMetrics, assignedTruthIDs);
    trackError{ind} = cumulativeTrackMetrics(errorMetrics);
    truthError{ind} = cumulativeTruthMetrics(errorMetrics);
end

wallClockTime = toc(a);
fprintf('  Sim loop done (%.1f s wall-clock).\n', wallClockTime);

%% ---- Package results ----
results.mapViewer     = mapViewer;
results.wallClockTime = wallClockTime;

results.truePoseGPSAll = truePoseGPSAll;
results.truePoseNEDAll = truePoseNEDAll;

results.fusedTrackLog  = fusedTrackLog;
results.radarTrackLog  = radarTrackLog;
results.adsbTrackLog   = adsbTrackLog;
results.adsbTrackLogNED = adsbTrackLogNED;

results.posRMSE  = posRMSE;
results.velRMSE  = velRMSE;
results.posANEES = posANEES;
results.velANEES = velANEES;

results.trackError = trackError;
results.truthError = truthError;
results.trackAssignmentSummary = trackAssignmentSummary;
results.truthAssignmentSummary = truthAssignmentSummary;

end % runFullSimulation


%% ========================================================================
%                          LOCAL HELPER FUNCTIONS
%  ========================================================================
% Self-contained copies — runFullSimulation does not rely on the main script.

function params = ecefParams()
params = struct('Frame','ECEF','IsCartesian',true,'AxesOrder','interleaved');
end

function params = nedParams()
params = struct('Frame','NED','IsCartesian',true,'AxesOrder','interleaved');
end

function tracksOut = ensureTimeAndParams(tracksIn, time, params)
tracksOut = tracksIn;
for k = 1:numel(tracksOut)
    tracksOut(k).UpdateTime      = double(time);
    tracksOut(k).StateParameters = params;
end
end

function s = toStructForMEX(tracks)
s = toStruct(tracks);
for k = 1:numel(s)
    s(k).TrackID     = uint32(s(k).TrackID);
    s(k).BranchID    = uint32(s(k).BranchID);
    s(k).SourceIndex = uint32(s(k).SourceIndex);
    s(k).Age         = uint32(s(k).Age);
    s(k).UpdateTime  = double(s(k).UpdateTime);
    s(k).State       = double(s(k).State);
    s(k).StateCovariance = double(s(k).StateCovariance);
    s(k).ObjectClassID   = double(s(k).ObjectClassID);
    s(k).ObjectClassProbabilities = double(s(k).ObjectClassProbabilities);
    s(k).TrackLogicState = double(s(k).TrackLogicState);
    s(k).IsConfirmed    = logical(s(k).IsConfirmed);
    s(k).IsCoasted      = logical(s(k).IsCoasted);
    s(k).IsSelfReported = logical(s(k).IsSelfReported);
    s(k).TrackLogic = 'Integrated';
    s(k).StateParameters = struct();
    attrs = s(k).ObjectAttributes;
    if isempty(attrs) || ~isfield(attrs, 'Callsign') || ~isfield(attrs, 'Category')
        s(k).ObjectAttributes = struct( ...
            'Callsign', repmat(' ', 1, 8), ...
            'Category', adsbCategory.No_Category_Information);
    end
end
end

function tracksObj = convertToObjectTrack(tracksStruct)
tracksObj = repmat(objectTrack, numel(tracksStruct), 1);
for i = 1:numel(tracksStruct)
    trackLogic = strtrim(tracksStruct(i).TrackLogic);
    tracksObj(i) = objectTrack( ...
        'TrackID', tracksStruct(i).TrackID, ...
        'BranchID', tracksStruct(i).BranchID, ...
        'SourceIndex', tracksStruct(i).SourceIndex, ...
        'UpdateTime', tracksStruct(i).UpdateTime, ...
        'Age', tracksStruct(i).Age, ...
        'State', tracksStruct(i).State, ...
        'StateCovariance', tracksStruct(i).StateCovariance, ...
        'StateParameters', tracksStruct(i).StateParameters, ...
        'ObjectClassID', tracksStruct(i).ObjectClassID, ...
        'ObjectClassProbabilities', tracksStruct(i).ObjectClassProbabilities, ...
        'TrackLogic', trackLogic, ...
        'TrackLogicState', tracksStruct(i).TrackLogicState, ...
        'IsConfirmed', tracksStruct(i).IsConfirmed, ...
        'IsCoasted', tracksStruct(i).IsCoasted, ...
        'IsSelfReported', tracksStruct(i).IsSelfReported, ...
        'ObjectAttributes', tracksStruct(i).ObjectAttributes ...
    );
end
end

function centralTrack = radar2central(radarTrack, mapOrigin)
centralTrack = objectTrack('State',zeros(6,1),'StateCovariance',eye(6));
centralTrack = syncTrack(centralTrack, radarTrack);

radarState = radarTrack.State;
[X,Y,Z] = ned2ecef(radarState(1),radarState(3),radarState(5), ...
    mapOrigin(1),mapOrigin(2),mapOrigin(3),wgs84Ellipsoid);
R = dcmecef2ned(mapOrigin(1), mapOrigin(2));
velos = R' * [radarState(2);radarState(4);radarState(6)];
centerState = [X, velos(1), Y, velos(2), Z, velos(3)];

radarStateCov = radarTrack.StateCovariance;
permute_idx   = [1, 3, 5, 2, 4, 6];
unpermute_idx = [1, 4, 2, 5, 3, 6];
P_perm = radarStateCov(permute_idx, permute_idx);
R6 = blkdiag(R', R');
P_rot = R6 * P_perm * R6';
centerStateCov = P_rot(unpermute_idx, unpermute_idx);

centralTrack.State = centerState;
centralTrack.StateCovariance = centerStateCov;
end

function radarTrack = central2radar(centralTrack, mapOrigin)
radarTrack = objectTrack('State',zeros(6,1),'StateCovariance',eye(6));
radarTrack = syncTrack(radarTrack, centralTrack);

centerState = centralTrack.State;
[N,E,D] = ecef2ned(centerState(1),centerState(3),centerState(5), ...
    mapOrigin(1),mapOrigin(2),mapOrigin(3),wgs84Ellipsoid);
[vN,vE,vD] = ecef2nedv(centerState(2),centerState(4),centerState(6), ...
    mapOrigin(1),mapOrigin(2));
radarState = [N, vN, E, vE, D, vD];

centerStateCov = centralTrack.StateCovariance;
R = dcmecef2ned(mapOrigin(1), mapOrigin(2));
permute_idx   = [1, 3, 5, 2, 4, 6];
unpermute_idx = [1, 4, 2, 5, 3, 6];
P_perm = centerStateCov(permute_idx, permute_idx);
R6 = blkdiag(R, R);
P_rot = R6 * P_perm * R6';
radarStateCov = P_rot(unpermute_idx, unpermute_idx);

radarTrack.State = radarState;
radarTrack.StateCovariance = radarStateCov;
end

function tr1 = syncTrack(tr1, tr2)
props = {'TrackID','BranchID','SourceIndex','UpdateTime','Age',...
    'StateParameters','ObjectClassID','IsConfirmed','IsCoasted',...
    'IsSelfReported','ObjectAttributes'};
for i = 1:numel(props)
    tr1.(props{i}) = tr2.(props{i});
end
end

function scenario = createScenario(tuningData, mapOrigin)
scenarioDuration = 100; % s (reduced from 200 for workshop speed)
scenario = trackingScenario(UpdateRate=1,StopTime=scenarioDuration,IsEarthCentered=true);
radarTower = platform(scenario, Position=mapOrigin); %#ok<NASGU>

beamwidthAz = 360;
fov = [beamwidthAz; 15];
updaterate = 1;
activeRadar = fusionRadarSensor(1,"No Scanning", ...
    UpdateRate=updaterate, ...
    FieldOfView=fov, ...
    AzimuthResolution=1.4, ...
    ReferenceRange=111e3, ...
    ReferenceRCS=0, ...
    RangeResolution=135, ...
    HasElevation=true, ...
    HasNoise=true, ...
    HasFalseAlarms=true, ...
    FalseAlarmRate=1e-7,...
    HasRangeRate=true, ...
    RangeRateLimits=[-600 600], ...
    MountingLocation=[0 0 -15], ...
    MountingAngles=[0 0 0], ...
    HasINS=true, ...
    DetectionCoordinates="Sensor spherical");

scenario.Platforms{1}.Sensors = activeRadar;
elFov = fov(2);
activeRadar.FieldOfView(2) = elFov+1e-3;

applehillPos = [42.300498, -71.349157 0];
platform(scenario, Position=applehillPos);

for i = 1:numel(tuningData)
    Pos = tuningData{i}.Position;
    time = seconds(tuningData{i}.Time);
    validIdx = time <= scenarioDuration;
    truncatedTime = time(validIdx);
    truncatedPos = Pos(validIdx, :);
    traj = geoTrajectory(truncatedPos, truncatedTime);
    platform(scenario, Trajectory=traj);
end
end

function transponderStruct = createTX(numPlats, gps)
transponderStruct(numPlats) = struct();
for i = 1:numPlats
    tailNumber = sprintf('MW%d', 2019 + i);
    transponderStruct(i).adsbTx = adsbTransponder( ...
        tailNumber, 'UpdateRate', 1, ...
        'GPS', gps, 'Category', adsbCategory.Large);
end
end

function varargout = splitSensorData(sensorData, stopTime, trackerInterval)
varargout = cell(1, nargout);
if iscell(sensorData)
    varargout = cellfun(@(s) splitOne(s,stopTime,trackerInterval), sensorData, UniformOutput=false);
else
    varargout{1} = splitOne(sensorData, stopTime, trackerInterval);
end
end

function data = splitOne(sensorData, stopTime, trackerInterval)
frameTime = 0;
ind = 1;
numFrames = ceil(stopTime/trackerInterval);
data = repmat(sensorData, numFrames, 1);
while frameTime < stopTime
    withinInterval = sensorData.LookTime >= frameTime & sensorData.LookTime < frameTime+trackerInterval;
    data(ind).LookTime = sensorData.LookTime(withinInterval);
    data(ind).LookAzimuth = sensorData.LookAzimuth(withinInterval);
    data(ind).LookElevation = sensorData.LookElevation(withinInterval);
    withinInterval = sensorData.DetectionTime >= frameTime & sensorData.DetectionTime < frameTime+trackerInterval;
    data(ind).DetectionTime = sensorData.DetectionTime(withinInterval);
    if isfield(sensorData,"Azimuth")
        data(ind).Azimuth = sensorData.Azimuth(1,withinInterval);
        data(ind).AzimuthAccuracy = sensorData.AzimuthAccuracy(1,withinInterval);
    end
    if isfield(sensorData,"Elevation")
        data(ind).Elevation = sensorData.Elevation(1,withinInterval);
        data(ind).ElevationAccuracy = sensorData.ElevationAccuracy(1,withinInterval);
    end
    if isfield(sensorData,"Range")
        data(ind).Range = sensorData.Range(1,withinInterval);
        data(ind).RangeAccuracy = sensorData.RangeAccuracy(1,withinInterval);
    end
    if isfield(sensorData,"RangeRate")
        data(ind).RangeRate = sensorData.RangeRate(1,withinInterval);
        data(ind).RangeRateAccuracy = sensorData.RangeRateAccuracy(1,withinInterval);
    end
    if isfield(sensorData,"TargetIndex")
        data(ind).TargetIndex = sensorData.TargetIndex(1,withinInterval);
    end
    frameTime = frameTime + trackerInterval;
    ind = ind + 1;
end
end
