%% 
%% System Integration and Test Simulation
%
% Architecture Overview
% ---------------------
% Coordinate Frames:
%   - Radar tracker operates in NED (North-East-Down) about mapOrigin
%   - trackFuser and ADS-B receiver operate in ECEF
%   - radar2central converts NED → ECEF; central2radar converts ECEF → NED
%   - dcmecef2ned (Aerospace Toolbox) provides the rotation matrix
%
% State Vector Layout:  [x vx y vy z vz]  (interleaved constant-velocity)
%   - Position indices: [1 3 5]    Velocity indices: [2 4 6]
%
% Data Flow:  Scenario → SensorData.mat → tracker (NED) → fuser (ECEF)
%             ADS-B receiver → fuser (ECEF)
%             Fused tracks → central2radar → NED metrics & OSPA
%
% Two-Phase Structure
% -------------------
% Phase 1: Configuration check — interpreted fuser, short loop (~15 steps),
%          every-step plotting so students can visually verify their config fix.
% Phase 2: Full simulation — calls runFullSimulation() with MEX fuser,
%          full duration, metrics, then plots results here.
%
%% Configure a simulation that uses the following:
% * Truth Locations - Field-collected ADSB data recorded with an RTL-SDR
% * Detections and Sensor-based Tracks:
% * Active radar (fusionRadarSensor)
% * ADSB data (gpsSensor and ADSBReceiver)
% * TrackFuser for C2 level tracks
% * OSPA Metric, trackAssignmentMetrics, and errorMetrics for track accuracy 
% analysis

close all force
clear

%% ========================================================================
%                    PHASE 1 — Configuration Visual Check
%  ========================================================================
% Run a short simulation with the INTERPRETED fuser so students can see 
% the effect of their adsbConfig changes in real time on the globe viewer.

%% Setup Scenario
% Import truth location of aircraft and convert to trajectories for simulation

mapOrigin = [42.39423231362 -70.95934958874 0]; % Also defined in radar2central, central2radar, and fusionAlgorithm.m
load("TrackingScenarioTruth.mat")

%% Scenario Creation includes configuring the Active Radar Sensor

scenario = helperCreateScenario(tuningData, mapOrigin);
mapViewer = trackingGlobeViewer('ReferenceLocation',mapOrigin,'Basemap','streets-dark');
campos(mapViewer, mapOrigin + [0 -0.4 1e5]);
drawnow;
plotScenario(mapViewer,scenario);
snapshot(mapViewer);

%% Secondary Sensor Configuration
% Add ADSB for secondary target detection

horAccuracy = 20;   % meters
vertAccuracy = 50;  % meters
velAccuracy = 0.4;  % m/s

gps = gpsSensor('PositionInputFormat','Geodetic',...
    'HorizontalPositionAccuracy',horAccuracy,...
    'VerticalPositionAccuracy',vertAccuracy,'VelocityAccuracy',velAccuracy);
numTargets = numel(scenario.Platforms)-2; % two platforms in scene are the sensor towers

%% Leverage a helper function to store GPS transponder in a structure.

transponderStruct = helperCreateTX(numTargets, gps);

%% Configure ADSB Receiver for SSR

adsbRx = adsbReceiver('ReceiverIndex',2);

%% Configure Tracker

[tracker,activeRadarSpec,targetSpec] = helperCreateTracker(scenario);

%% Configure Track Fuser
% You fuse radar tracks with ADS-B tracks obtained from the ADS-B receiver. 
% To do this, you configure a central |trackFuser| object.

% Define the radar source configuration
radarConfig = fuserSourceConfiguration('SourceIndex',1,...
    'IsInitializingCentralTracks',false,...
    'CentralToLocalTransformFcn',@central2radar,...
    'LocalToCentralTransformFcn',@radar2central);

%% *Step 1 - Adjust Coordinate Transform Configuration to Match Data*
% *Directions:*
% 
% By running the script as configured, you should see that the adsbTracks are 
% placed far from the truth positions of the targets. By adjusting the "CentralToLocalTransformFcn" 
% and "LocalToCentralTransformFcn" to use the correct coordinate transforms, this 
% should fix the issue.
% 
% Note - While I have given you the coordinate transforms, they are only flipped 
% for the adsbConfig.  In practice, you may have to generate this transforms yourself.  
% It is powerful to use the visualization tools to check that you have done this 
% setup correctly while configuring the functions.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% type "doc fuserSourceConfiguration" in the Command Window to find the documentation page for this function
% ADJUST THE adsbConfig's "CentralToLocalTransformFcn" and "LocalToCentralTransformFcn" to perform the
% transforms in the correct direction, ie opposite of the radarConfig:

% Define the adsb source configuration
adsbConfig = fuserSourceConfiguration('SourceIndex',2,...
    'IsInitializingCentralTracks',true,...
    'CentralToLocalTransformFcn',@radar2central,...
    'LocalToCentralTransformFcn',@central2radar); % This is solution - revert for the example

%% Instantiate the track fuser (interpreted — used in Phase 1 only)

fuser = trackFuser('FuserIndex',3,'MaxNumSources',2,...
    "AssignmentThreshold",[75 250],...
    "StateFusion",'Intersection',...
    'StateFusionParameters','trace',... 
    'ConfirmationThreshold',[2 3],... 
    'DeletionThreshold',[4 4],... 
    'SourceConfigurations',{radarConfig;adsbConfig});

%% Load Pre-recorded Sensor Data

load('SensorData.mat');

%% Reset Scenario, Tracker, Viewer, and Transponders

clear(mapViewer);
restart(scenario);
release(tracker);

%% Split dataset for easier parsing within simulation

trackerUpdateInterval = 1; % seconds
[activeRadarData] = helperSplitSensorData(sensorData, scenario.StopTime, trackerUpdateInterval);

%% Phase 1 — Short Visual-Check Loop
% Run a small number of steps with the INTERPRETED fuser so students can
% see whether ADS-B tracks are landing on or far from the truth positions.
% If the adsbConfig is wrong, tracks will appear displaced on the globe.

phase1Steps = 15;  % enough to see ADS-B and fused tracks appear
fprintf('\n--- Phase 1: Running %d steps with interpreted fuser for visual check ---\n', phase1Steps);
a1 = tic;

% Reset for Phase 1
clear(mapViewer);
restart(scenario);
release(tracker);
reset(adsbRx);
for i = 1:numTargets
    reset(transponderStruct(i).adsbTx);
end
plotPlatform(mapViewer,[scenario.Platforms{3:end}],TrajectoryMode="Full");

% Labels and colors
adsblabel = "       ADS-B";
radarlabel = "  Radar";
fusedlabel = string(sprintf('%s\n',"","Fused"));
adsbclr = [183 70 255]/255;
radarclr = [255 255 17]/255;
fusedclr = [255 105 41]/255;

wgs84 = wgs84Ellipsoid('meter');

for ind = 1:phase1Steps
    advance(scenario);

    % Coverage
    covcon = coverageConfig(scenario);
    plotCoverage(mapViewer,covcon,"ECEF",Color=[0 1 0])

    % Truth positions
    for i = 1:numel(scenario.Platforms)
        poseStruct = pose(scenario.Platforms{i},'CoordinateSystem','Geodetic');
        poseStruct.PlatformID = scenario.Platforms{i}.PlatformID;
        truePoseGPS(i) = poseStruct;
    end
    truePosePlot = platformPoses(scenario);
    plotPlatform(mapViewer, truePosePlot, 'ECEF');
    plotActiveRadarData(mapViewer, activeRadarSpec, activeRadarData(ind));

    time = (ind - 1) * trackerUpdateInterval;

    % ADS-B processing
    adsbTracks     = objectTrack.empty;
    adsbTracksPlot = objectTrack.empty;
    radarTracks    = objectTrack.empty;
    fusedTracks    = objectTrack.empty;

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

    % Convert to NED for globe viewer
    for i = 1:length(adsbTracks)
        adsbTracksPlot(i) = central2radar(adsbTracks(i));
    end

    % Radar processing
    radarTracksStruct = trackingAlgorithm_mex({activeRadarData(ind)},targetSpec,activeRadarSpec);
    radarTracks = convertToObjectTrack(radarTracksStruct);
    radarTracks = ensureTimeAndParams(radarTracks, time, nedParams());

    % Fusion — INTERPRETED fuser using student's adsbConfig
    if isLocked(fuser) || ~isempty([adsbTracks;radarTracks])
        fusedTracks = fuser([adsbTracks;radarTracks], time);
    end

    % Convert fused tracks (ECEF) to NED for globe viewer plotting
    fusedTracksNED = objectTrack.empty;
    for i = 1:length(fusedTracks)
        fusedTracksNED(i) = central2radar(fusedTracks(i));
    end

    % Force column orientation — interpreted fuser returns row vectors,
    % but helperPlotTracks vertcats with radarTracks (column).
    fusedTracksNED = fusedTracksNED(:);

    % Plot all tracks (fused tracks converted to NED to match viewer frame)
    mapViewer = helperPlotTracks(3,mapViewer,adsbTracksPlot,radarTracks,fusedTracksNED,...
        adsblabel,adsbclr,radarlabel,radarclr,fusedlabel,fusedclr);

    drawnow;
end

fprintf('Phase 1 complete (%.1f s).  Check the globe viewer.\n', toc(a1));
fprintf('  → If ADS-B/fused tracks are displaced from truth, fix adsbConfig above and re-run.\n');
fprintf('  → If tracks overlay the truth, your config is correct — proceed to Phase 2.\n\n');
snapshot(mapViewer);

%% ========================================================================
%                    PHASE 2 — Full Simulation with MEX Fuser
%  ========================================================================
% Phase 2 uses the MEX-compiled fusionAlgorithm (which has its own correct
% hardcoded config) for the full-duration simulation with metrics.
% This is wrapped in runFullSimulation() for a clean, single function call.

% Clean up the interpreted fuser from Phase 1
clear fuser  % <-- Phase 1 fuser is no longer needed

%% Run Full Simulation
% runFullSimulation re-creates the scenario, tracker, and fuser from scratch
% using the MEX-compiled fusionAlgorithm_mex, runs the full simulation, and
% returns all logged data and metrics.

fprintf('--- Phase 2: Running full simulation with MEX fuser ---\n');
results = runFullSimulation(tuningData, mapOrigin);
fprintf('Phase 2 complete (%.1f s wall-clock).\n\n', results.wallClockTime);

%% Take Snapshot of Viewer (from Phase 2)
snapshot(results.mapViewer);

%% ========================================================================
%                    POST-SIMULATION VISUALIZATION
%  ========================================================================
% All data comes from the results struct returned by runFullSimulation.

%% Compute OSPA
ospa = trackOSPAMetric('CutoffDistance',500,'Distance','posabserr');

% Filter truths to moving targets only (drop first 2 platforms = sensor towers)
truthsFilteredNEDAll = cell(size(results.truePoseNEDAll));
for i = 1:numel(results.truePoseNEDAll)
    tp = results.truePoseNEDAll{i};
    if numel(tp) > 2
        tp = tp(3:end);
    else
        tp = tp([]);
    end
    truthsFilteredNEDAll{i} = tp;
end

% Radar OSPA
N_ospa = numel(results.radarTrackLog);
radarospa = nan(N_ospa,1); radarLoc = nan(N_ospa,1); radarCard = nan(N_ospa,1);
for i = 1:N_ospa
    tracks = results.radarTrackLog{i};
    truths = truthsFilteredNEDAll{i};
    if isempty(tracks) && isempty(truths), continue; end
    [radarospa(i), radarLoc(i), radarCard(i)] = ospa(tracks, truths);
end

% ADS-B OSPA
N_adsb = numel(results.adsbTrackLogNED);
adsbospa = nan(N_adsb,1); adsbLoc = nan(N_adsb,1); adsbCard = nan(N_adsb,1);
reset(ospa);
for i = 1:N_adsb
    tracks = results.adsbTrackLogNED{i};
    truths = truthsFilteredNEDAll{i};
    if isempty(tracks) && isempty(truths), continue; end
    [adsbospa(i), adsbLoc(i), adsbCard(i)] = ospa(tracks, truths);
end

% Fused OSPA
N_fused = numel(results.fusedTrackLog);
fusedospa = nan(N_fused,1); fuseLoc = nan(N_fused,1); fuseCard = nan(N_fused,1);
reset(ospa);
for i = 1:N_fused
    tracks = results.fusedTrackLog{i};
    truths = truthsFilteredNEDAll{i};
    if isempty(tracks) && isempty(truths), continue; end
    [fusedospa(i), fuseLoc(i), fuseCard(i)] = ospa(tracks, truths);
end

%% Plot OSPA (Combined)
radarclr = [255 255 17]/255;
adsbclr  = [183 70 255]/255;
fusedclr = [255 105 41]/255;

figure
hold on
plot((0:(N_ospa-1))/60, radarospa, "Color",radarclr);
plot((0:(N_adsb-1))/60, adsbospa, "Color",adsbclr, 'LineWidth',2);
plot((0:(N_fused-1))/60, fusedospa, "Color",fusedclr);
l=legend('Radar','ADS-B','Radar + ADS-B');
l.Color = [0.1 0.1 0.1];
l.TextColor = [1 1 1];
xlabel('Time (min)')
ylabel('OSPA')
ax = gca;
grid on; box on;
ax.Color = [0.1 0.1 0.1];
ax.GridColor = [1 1 1];

%% Plot OSPA (Separate Subplots)
figure
subplot(3,1,1)
plot((0:(N_ospa-1))/60, radarospa, "Color",radarclr);
l=legend('Radar');
ylabel('OSPA')

subplot(3,1,2)
plot((0:(N_adsb-1))/60, adsbospa, "Color",adsbclr, 'LineWidth',2);
l2=legend('ADS-B');
ylabel('OSPA')

subplot(3,1,3)
plot((0:(N_fused-1))/60, fusedospa, "Color",fusedclr);
l3=legend('Radar + ADS-B');
ylabel('OSPA')

l.Color = [0.1 0.1 0.1]; l2.Color = [0.1 0.1 0.1]; l3.Color = [0.1 0.1 0.1];
l.TextColor = [1 1 1]; l2.TextColor = [1 1 1]; l3.TextColor = [1 1 1];
xlabel('Time (min)')
ax = gca;
grid on; box on;
ax.Color = [0.1 0.1 0.1];
ax.GridColor = [1 1 1];

%% Plot RMS Values
figure;
timeVec = 1:length(results.posRMSE);
subplot(2,2,1)
scatter(timeVec,results.posRMSE,'filled');
xlabel('Time step');
ylabel('RMS Error in Position (m)');
grid('on');

subplot(2,2,2)
scatter(timeVec,results.velRMSE,'filled');
xlabel('Time step');
ylabel('RMS Error in Velocity (m/s)');
grid('on');

subplot(2,2,3)
scatter(timeVec,results.posANEES,'filled');
xlabel('Time step');
ylabel('Average Normalized Error in Position');
grid('on');

subplot(2,2,4)
scatter(timeVec,results.velANEES,'filled');
xlabel('Time step');
ylabel('Average Normalized Error in Velocity');
grid('on');

sgtitle('Cumulative errors for all tracks','FontWeight','bold');

save('SimulatedData.mat')

%% ========================================================================
%                         LOCAL HELPER FUNCTIONS
%  ========================================================================
% These are shared by Phase 1 (above) and used by the main script.
% runFullSimulation.m has its own copies of what it needs.

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

function scenario = helperCreateScenario(tuningData, mapOrigin)
scenarioDuration = 100; % s (reduced from 200 for workshop speed)
scenario = trackingScenario(UpdateRate=1,StopTime=scenarioDuration,IsEarthCentered=true);
radarTower = platform(scenario, Position=mapOrigin);

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

radarTower.Sensors = activeRadar;
elFov = fov(2);
activeRadar.FieldOfView(2) = elFov+1e-3;

applehillPos = [42.300498, -71.349157 0];
ADSBTower = platform(scenario, Position=applehillPos); %#ok<NASGU>

for i = 1:numel(tuningData)
    Pos = tuningData{i}.Position;
    time = seconds(tuningData{i}.Time);
    validIdx = time <= scenarioDuration;
    truncatedTime = time(validIdx);
    truncatedPos = Pos(validIdx, :);
    traj = geoTrajectory(truncatedPos,truncatedTime);
    platform(scenario,Trajectory=traj);
end
end

function transponderStruct = helperCreateTX(numPlats, gps)
    transponderStruct(numPlats) = struct();
    for i = 1:numPlats
        tailNumber = sprintf('MW%d', 2019 + i);
        transponderStruct(i).adsbTx = adsbTransponder( ...
            tailNumber, ...
            'UpdateRate', 1, ...
            'GPS', gps, ...
            'Category', adsbCategory.Large);
    end
end

function sensorData = helperRecordSensorData(scenario) %#ok<DEFNU>
if isa(scenario,"trackingScenarioRecording")
    recording = scenario.RecordedData;
elseif isa(scenario,"struct")
    recording = scenario;
elseif isa(scenario,"trackingScenario") || isa(scenario,"radarScenario")
    seed = 2024;
    disp("Recording the scenario to get sensor data. This may take several minutes.");
    recording = record(scenario,"Rotmat",IncludeSensors=true,RecordingFormat="struct",InitialSeed=seed);
else
    error("This example utility function can only be used with a trackingScenario a trackingScenarioRecording");
end

if ~isfield(recording,"SensorConfigurations") || ~isfield(recording,"Detections")
    error("Scenario recording must contain sensor configurations and detections to record sensor data");
end
sensors = recording(1).SensorConfigurations;
detBuffer = vertcat(recording.Detections);
for i = 1:numel(recording)
    for j = 1:numel(recording(i).SensorConfigurations)
        recording(i).SensorConfigurations(j).Time = recording(i).SimulationTime;
    end
end
configBuffer = reshape([recording.SensorConfigurations],[],1);
sensorData = cell(1,numel(sensors));
for k = 1:numel(sensors)
    sensor = sensors(k);
    sensorIdx = sensor.SensorIndex;
    sensorMountingAngles = eye(3);
    detSensorIndex = cellfun(@(x)x.SensorIndex,detBuffer);
    dets = detBuffer(detSensorIndex == sensorIdx);
    cfgSensorIndex = arrayfun(@(x)x.SensorIndex, configBuffer);
    cfgs = configBuffer(cfgSensorIndex == sensorIdx);
    if sensor.MeasurementParameters(1).HasRange
        sensorData{k} = recordMonostaticData(sensorMountingAngles, dets, cfgs);
    else
        disp('no ESM sensor, not recording')
    end
end
end

function activeRadarData = recordMonostaticData(sensorMountingAngles, detections, cfgs)
lookMountRot = cell(numel(cfgs),1);
for i = 1:numel(cfgs)
    if cfgs(i).MeasurementParameters(1).IsParentToChild
        lookMountRot{i} = cfgs(i).MeasurementParameters(1).Orientation;
    else
        lookMountRot{i} = cfgs(i).MeasurementParameters(1).Orientation';
    end
end
beamRot = cellfun(@(x)x*sensorMountingAngles',lookMountRot,UniformOutput=false);
ypr = cellfun(@(x)fusion.internal.frames.rotmat2ypr(x,"degrees"),beamRot,UniformOutput=false);
lookTime = arrayfun(@(x)x.Time,cfgs);
lookAzimuth = cellfun(@(x)x(1),ypr);
lookElevation = cellfun(@(x)-x(2),ypr);

detectionTime = cellfun(@(x)x.Time,detections);
azimuth = cellfun(@(x)x.Measurement(1),detections);
elevation = cellfun(@(x)x.Measurement(2),detections);
range = cellfun(@(x)x.Measurement(3),detections);
rangeRate = cellfun(@(x)x.Measurement(4),detections);
azimuthAccuracy = cellfun(@(x)x.MeasurementNoise(1,1),detections);
elevationAccuracy = cellfun(@(x)x.MeasurementNoise(2,2),detections);
rangeAccuracy = cellfun(@(x)x.MeasurementNoise(3,3),detections);
rangeRateAccuracy = cellfun(@(x)x.MeasurementNoise(4,4),detections);

activeRadarData = struct(LookTime=lookTime(:)',LookAzimuth=lookAzimuth(:)', ...
    LookElevation=lookElevation(:)', DetectionTime=detectionTime(:)', ...
    Azimuth=azimuth(:)',Elevation=elevation(:)', Range=range(:)', ...
    RangeRate=rangeRate(:)',AzimuthAccuracy=sqrt(azimuthAccuracy(:)'), ...
    ElevationAccuracy=sqrt(elevationAccuracy(:)'),RangeAccuracy=sqrt(rangeAccuracy(:)'), ...
    RangeRateAccuracy=sqrt(rangeRateAccuracy(:)'));
end

function varargout=helperSplitSensorData(sensorData,stopTime,trackerInterval)
varargout = cell(1,nargout);
if iscell(sensorData)
    varargout = cellfun(@(s) splitOneSensorData(s,stopTime,trackerInterval), sensorData, UniformOutput=false);
else
    varargout{1} = splitOneSensorData(sensorData,stopTime,trackerInterval);
end
end

function data=splitOneSensorData(sensorData,stopTime,trackerInterval)
frameTime = 0;
ind = 1;
numFrames = ceil(stopTime/trackerInterval);
data = repmat(sensorData,numFrames,1);
while frameTime < stopTime
    withinInterval = sensorData.LookTime >= frameTime & sensorData.LookTime < frameTime+trackerInterval;
    data(ind).LookTime = sensorData.LookTime(withinInterval);
    data(ind).LookAzimuth = sensorData.LookAzimuth(withinInterval);
    data(ind).LookElevation = sensorData.LookElevation(withinInterval);
    withinInterval = sensorData.DetectionTime >=frameTime & sensorData.DetectionTime < frameTime+trackerInterval;
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

function platformPoses = helperSavePlatformPoses(scenario,trackerInterval) %#ok<DEFNU>
endind = ceil(scenario.StopTime/trackerInterval);
samplePose = struct(PlatformID=1,ClassID=0,Position=zeros(1,3),Velocity=zeros(1,3),Acceleration=zeros(1,3),Orientation=eye(3),AngularVelocity=zeros(1,3));
platformPoses = repmat(samplePose,numel(scenario.Platforms),endind);
sampleTimes = trackerInterval*(1:endind);

for p = 1:numel(scenario.Platforms)
    thisPlatform = scenario.Platforms{p};
    trajectory = thisPlatform.Trajectory;
    if isa(trajectory,"waypointTrajectory")
        [position,orientation,velocity,acceleration,angularVelocity] = lookupPose(trajectory,sampleTimes);
        orientation = rotmat(orientation,"frame");
        for i = 1:endind
            platformPoses(p,i) = struct(PlatformID=thisPlatform.PlatformID,ClassID=thisPlatform.ClassID, ...
                Position=position(i,:),Velocity=velocity(i,:),Acceleration=acceleration(i,:), ...
                Orientation=orientation(:,:,i),AngularVelocity=angularVelocity(i,:));
        end
    else
        platformPoses(p,:) = repmat(struct(PlatformID=thisPlatform.PlatformID,ClassID=thisPlatform.ClassID, ...
            Position=thisPlatform.Position,Velocity=zeros(1,3),Acceleration=zeros(1,3),Orientation=thisPlatform.Orientation, ...
            AngularVelocity=zeros(1,3)),1,endind);
    end
end
end

function centralTrack = radar2central(radarTrack)
mapOrigin = [42.39423231362 -70.95934958874 0];
centralTrack = objectTrack('State',zeros(6,1),...
    'StateCovariance',eye(6));
centralTrack = syncTrack(centralTrack,radarTrack);

radarState = radarTrack.State;
[X,Y,Z] = ned2ecef(radarState(1),radarState(3),radarState(5),mapOrigin(1),mapOrigin(2),mapOrigin(3),wgs84Ellipsoid); 
R = dcmecef2ned(mapOrigin(1), mapOrigin(2));
velos = R' * [radarState(2);radarState(4);radarState(6)];
centerState = [X, velos(1), Y, velos(2), Z, velos(3)];

radarStateCov = radarTrack.StateCovariance;
permute_idx = [1, 3, 5, 2, 4, 6];
unpermute_idx = [1, 4, 2, 5, 3, 6];
P_perm = radarStateCov(permute_idx, permute_idx);
R6 = blkdiag(R', R');
P_rot = R6 * P_perm * R6';
centerStateCov = P_rot(unpermute_idx, unpermute_idx);

centralTrack.State = centerState;
centralTrack.StateCovariance = centerStateCov;
end

function radarTrack = central2radar(centralTrack)
mapOrigin = [42.39423231362 -70.95934958874 0];
radarTrack = objectTrack('State',zeros(6,1),...
    'StateCovariance',eye(6));
radarTrack = syncTrack(radarTrack,centralTrack);

centerState = centralTrack.State;
[N,E,D] = ecef2ned(centerState(1),centerState(3),centerState(5),mapOrigin(1),mapOrigin(2),mapOrigin(3),wgs84Ellipsoid);
[vN,vE,vD] = ecef2nedv(centerState(2),centerState(4),centerState(6),mapOrigin(1),mapOrigin(2));
radarState = [N, vN, E, vE, D, vD];

centerStateCov = centralTrack.StateCovariance;
R = dcmecef2ned(mapOrigin(1), mapOrigin(2));
permute_idx = [1, 3, 5, 2, 4, 6];
unpermute_idx = [1, 4, 2, 5, 3, 6];
P_perm = centerStateCov(permute_idx, permute_idx);
R6 = blkdiag(R, R);
P_rot = R6 * P_perm * R6';
radarStateCov = P_rot(unpermute_idx, unpermute_idx);

radarTrack.State = radarState;
radarTrack.StateCovariance = radarStateCov;
end

function tr1 = syncTrack(tr1,tr2)
props = {...
    'TrackID', ...
    'BranchID', ...
    'SourceIndex', ...
    'UpdateTime', ...
    'Age', ...
    'StateParameters', ...
    'ObjectClassID', ...
    'IsConfirmed', ...
    'IsCoasted', ...
    'IsSelfReported', ...
    'ObjectAttributes'};
for i = 1:numel(props)
    tr1.(props{i}) = tr2.(props{i});
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

function trackStruct = fixTypesForMEX(trackStruct) %#ok<DEFNU>
if isempty(trackStruct)
    return;
end
for i = 1:length(trackStruct)
    if isfield(trackStruct(i), 'SourceIndex')
        srcIdx = trackStruct(i).SourceIndex; %#ok<NASGU>
    else
        warning('Track %d missing SourceIndex!', i);
    end
    if isfield(trackStruct(i), 'ObjectClassID')
        trackStruct(i).ObjectClassID = double(trackStruct(i).ObjectClassID);
    end
    if isfield(trackStruct(i), 'ObjectClassProbabilities')
        trackStruct(i).ObjectClassProbabilities = double(trackStruct(i).ObjectClassProbabilities);
    end
    if isfield(trackStruct(i), 'TrackLogicState')
        state = double(trackStruct(i).TrackLogicState);
        if any(state ~= 0 & state ~= 1)
            state = double(state >= 0.5);
        end
        trackStruct(i).TrackLogicState = state;
    end
    if isfield(trackStruct(i), 'ObjectAttributes')
        attrs = trackStruct(i).ObjectAttributes;
        if isempty(attrs) || ~isfield(attrs, 'Callsign') || ~isfield(attrs, 'Category')
            trackStruct(i).ObjectAttributes = struct(...
                'Callsign', repmat(' ', 1, 8), ...
                'Category', adsbCategory.No_Category_Information);
        end
    end
    if isfield(trackStruct(i), 'TrackLogic')
        trackStruct(i).TrackLogic = 'Integrated';
    end
end
end
