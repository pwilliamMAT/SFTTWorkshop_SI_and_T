function results = helperRunFullSimulation(tuningData, mapOrigin,fuser)
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
close all force;
a = tic;
recordDataFlag = 0; % Set to 0 to load data, 1 to record scenario

%% ---- Scenario ----
truncatedStopTime = 60; % Limit Simulation time (s)
scenario = helperCreateScenario(tuningData, mapOrigin,truncatedStopTime);

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
PDrop = 0.3;
adsbRange = 2e5;

transponderStruct = helperCreateTX(numTargets, gps);
adsbRx = adsbReceiver('ReceiverIndex',2);

%% ---- Tracker ----
[tracker, activeRadarSpec, targetSpec] = helperCreateTracker(scenario); %#ok<ASGLU>

if recordDataFlag == 1
    % Record Scenario Sensor Data
    sensorData = helperRecordSensorData(scenario);
else
    %Load sensor data ----
    load('SensorData.mat','sensorData');
end

% Reset scenario, viewer, transponders
clear(mapViewer);
restart(scenario);
release(tracker);

trackerUpdateInterval = 1;
% Note scenario.StopTime matches the truncated duration indicated at
% scenario initialization above in createScenario()
activeRadarData = helperSplitSensorData(sensorData, scenario.StopTime, trackerUpdateInterval);

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
plotPlatform(mapViewer,[scenario.Platforms{3:end}],TrajectoryMode="Full"); % Omit Plotting Platforms without Trajectories

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
NEDPlat = scenario.Platforms{1};
for ind = 1:numStepsToRun
    advance(scenario);

    plotThisStep = (mod(ind, plotInterval) == 0) || ind == 1 || ind == numStepsToRun;

    % Coverage
    covcon = coverageConfig(scenario);
    if ind == 1 %plotThisStep
        plotCoverage(mapViewer, covcon, "ECEF") %, Color=[0 1 0] - Green, if desired
    end

    % Truth positions (LLA + NED)
    for i = 1:numel(scenario.Platforms)
        poseStruct = pose(scenario.Platforms{i},'CoordinateSystem','Geodetic');
        poseStruct.PlatformID = scenario.Platforms{i}.PlatformID;
        truePoseGPS(i) = poseStruct;

    end

    truePoseNED = targetPoses(NEDPlat); % Note Indexing is shifted by one, omits the reference platform

    truePoseGPSAll{ind} = truePoseGPS;
    truePoseNEDAll{ind} = truePoseNED;
    truePosePlot = platformPoses(scenario);

    if plotThisStep
        plotPlatform(mapViewer, truePosePlot, 'ECEF');
        helperPlotActiveRadarData(mapViewer, activeRadarSpec, activeRadarData(ind));
    end

    time = scenario.SimulationTime;

    % Init track arrays
    adsbTracks     = objectTrack.empty;
    adsbTracksNED = objectTrack.empty;
    radarTracks    = objectTrack.empty;
    fusedTracks    = objectTrack.empty;

    % --- ADS-B ---
    % Generate ADSB Messages
    adsbMessages = generateTemplateADSBMessage;
    adsbMessages(1)=[]; % Clear Values
    %k = 0;
    for i = 3:numel(truePoseGPS)
        position = truePoseGPS(i).Position;
        velocity = truePoseGPS(i).Velocity;
        isValid = all(~isnan(position));
        
        % Augment PDrop according to range
        PDropClose = 0.02;   % 5% dropout at close range
        PDropFar = 0.3;     % 30% dropout at max range
        cartDistance = ADSBTargetRange(mapOrigin, position);
        range_m = norm(cartDistance);
        rangeFactor = range_m / adsbRange;
        actualPDrop  = PDropClose + (PDropFar - PDropClose) * rangeFactor;
        keepMsg = rand >= actualPDrop;

        if isValid && keepMsg
            msg = transponderStruct(i-2).adsbTx(position,velocity);
            msg.Time = time;
            adsbMessages = [adsbMessages; msg];
        end
    end

    % Only call adsbRx if there are messages
    if ~isempty(adsbMessages)
        adsbTracks = adsbRx(adsbMessages, time);
        adsbTracks = adsbTracks';
    else
        adsbTracks = objectTrack.empty;
    end
    
    clear adsbMessages
    adsbTracks = helperEnsureTimeAndParams(adsbTracks, time, helperEcefParams());

    for i = 1:length(adsbTracks)
        adsbTracksNED(i) = helperEcef2nedTrack(adsbTracks(i));
    end

    % --- Radar ---
    radarTracksStruct = trackingAlgorithm_mex({activeRadarData(ind)}, targetSpec, activeRadarSpec);
    radarTracks = helperConvertToObjectTrack(radarTracksStruct);
    radarTracks = helperEnsureTimeAndParams(radarTracks, time, helperNedParams(mapOrigin));

    % --- Fusion (MEX) ---
    if ~isempty([adsbTracksNED';radarTracks])
        trackObj    = [adsbTracksNED';radarTracks];
        % % MEX
        % trackStruct = helperToStructForMEX(trackObj);
        % fusedStructs = fusionAlgorithm_mex(trackStruct, time);
        % if ~isempty(fusedStructs)
        %     fusedTracks = helperConvertToObjectTrack(fusedStructs);
        % end
        fusedTracks = fuser(trackObj, time);
    end

    % % Check truth versus track totals
    % numTruthTargets =  length(truePosePlot)-2;  % 2 stationary towers, not targets
    % numFusedTracks = length(fusedTracks);
    % numConfirmedFused = sum([fusedTracks.IsConfirmed]);
    % 
    % fprintf('Time: %.1f | Truth: %d | Fused: %d | Confirmed: %d | Delta: %d\n', ...
    %     time, numTruthTargets, numFusedTracks, numConfirmedFused, ...
    %     numFusedTracks - numTruthTargets);

    % --- Plot ---
    if plotThisStep
        mapViewer = helperPlotTracks(3,mapViewer,adsbTracksNED,radarTracks,fusedTracks,...
            adsblabel,adsbclr,radarlabel,radarclr,fusedlabel,fusedclr);
    end

    fusedTrackLog{ind}  = fusedTracks;
    radarTrackLog{ind}  = radarTracks;
    adsbTrackLog{ind}   = adsbTracks;
    adsbTrackLogNED{ind} = adsbTracksNED;

    % --- Metrics ---
    truthsForMetrics = truePoseNED;
    % if numel(truthsForMetrics) > 2
    %     truthsForMetrics = truthsForMetrics(3:end);
    % else
    %     truthsForMetrics = truthsForMetrics([]);
    % end
    stationaryIDs = [1, 2];  % Adjust based on your scenario
    truthsForMetrics = truePoseNED(~ismember([truePoseNED.PlatformID], stationaryIDs));

    % Fused
    [trackAssignmentSummary(ind), truthAssignmentSummary(ind)] = assignmentMetrics(fusedTracks, truthsForMetrics);
    [assignedTrackIDs, assignedTruthIDs] = currentAssignment(assignmentMetrics);
    [posRMSE(ind), velRMSE(ind), posANEES(ind), velANEES(ind)] = errorMetrics(fusedTracks, assignedTrackIDs, truthsForMetrics, assignedTruthIDs);
    trackError{ind} = cumulativeTrackMetrics(errorMetrics);
    truthError{ind} = cumulativeTruthMetrics(errorMetrics);
    assignedTruthIDsPerTime{ind} = assignedTruthIDs;

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

results.assignedTruthIDs = assignedTruthIDsPerTime;

end % runFullSimulation

function templateADSBMessage = generateTemplateADSBMessage
templateADSBMessage = struct('ICAO', 'MW2020',...
                         'Time', '',...
                     'Category','',...
                     'Callsign','',...
                     'Latitude',NaN,...
                    'Longitude',NaN,...
                     'Altitude',NaN,...
                       'Veast',NaN,...
                       'Vnorth',NaN,...
                   'ClimbRate',NaN,...
                      'Heading',NaN,...
                  'NACPosition', NaN,...
    'GeometricVerticalAccuracy',NaN,...
                  'NACVelocity',NaN);
end

function cartDistance = ADSBTargetRange(stationsLLA, pos) %cartDistance
cartDistance = fusion.internal.frames.lla2ecef(stationsLLA) - fusion.internal.frames.lla2ecef(pos);
%flag = any(vecnorm(cartDistance,2,2) < range);
end