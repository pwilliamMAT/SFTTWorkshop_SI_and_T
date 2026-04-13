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

transponderStruct = helperCreateTX(numTargets, gps);
adsbRx = adsbReceiver('ReceiverIndex',2);
adsbRange = 100e3; % range in meters (50e3 results in some missed targets)

% Configure a Gilbert–Elliott dropout model tuned for 1 Hz
dropper = helperMakeAdsbDropper('model','gilbert', ...
                          'pDropG', 0.01, ...   % Good: 1% per message dropped
                          'pDropB', 0.60, ...   % Bad : 60% per message dropped
                          'pGB',    0.004, ...  % Good->Bad per message (1Hz)
                          'pBG',    0.33);      % Bad->Good per message (1Hz)

% Optional: for reproducibility during runs
% rng(42,'twister');

% 1 Hz sample time
analytics = helperMakeAdsbDropoutAnalytics(1);  % Ts = 1 second

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
fusedlabel = string(sprintf('%s\n',"","C2 Level Track"));
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
        plotCoverage(mapViewer, covcon, "ECEF"); %, Color=[0 1 0]
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
    adsbTracksPlot = objectTrack.empty;
    radarTracks    = objectTrack.empty;
    fusedTracks    = objectTrack.empty;

    % --- ADS-B ---
    for i = 3:numel(truePoseGPS) % Omit Towers
        position = truePoseGPS(i).Position;
        velocity = truePoseGPS(i).Velocity;
        if ~isnan(position)
            adsbMessages(i-2,1) = transponderStruct(i-2).adsbTx(position,velocity);
        end
    end
    
        
    % Messages can be received within range of the surveillance station
    adsbRangeFlag = helperIsWithinRange(mapOrigin, truePoseGPS, adsbRange);
    receivedADSB_pre = adsbMessages(adsbRangeFlag);

    % Apply dropout model (bursty, per-ICAO)
    [receivedADSB_post, dropper] = helperStepAdsbDropper(dropper, receivedADSB_pre, time);


    % Update analytics (pass both pre- and post-drop lists)
    analytics = helperStepAdsbDropoutAnalytics(analytics, receivedADSB_pre, receivedADSB_post, time);

    adsbTracks = adsbRx(receivedADSB_post, time);
    %adsbTracks = adsbRx(adsbMessages, time); % Works without isWithinRange
    %function above
    adsbTracks = adsbTracks';
    clear adsbMessages
    adsbTracks = helperEnsureTimeAndParams(adsbTracks, time, helperEcefParams());

    for i = 1:length(adsbTracks)
        adsbTracksPlot(i) = helperEcef2nedTrack(adsbTracks(i));
    end

    % --- Radar ---
    radarTracksStruct = trackingAlgorithm_mex({activeRadarData(ind)}, targetSpec, activeRadarSpec);
    radarTracks = helperConvertToObjectTrack(radarTracksStruct);
    radarTracks = helperEnsureTimeAndParams(radarTracks, time, helperNedParams());

    % --- Fusion (MEX) ---
    if ~isempty([adsbTracksPlot';radarTracks])
        trackObj    = [adsbTracksPlot';radarTracks];
        trackStruct = helperToStructForMEX(trackObj);
        %fusedStructs = fusionAlgorithm_mex(trackStruct, time);
        fusedStructs = fusionAlgorithm(trackStruct, time); % DEBUG: Omit once fixed
        if ~isempty(fusedStructs)
            fusedTracks = helperConvertToObjectTrack(fusedStructs);
        end
    end

    % --- Plot ---
    if plotThisStep
        mapViewer = helperPlotTracks(3,mapViewer,adsbTracksPlot,radarTracks,fusedTracks,...
            adsblabel,adsbclr,radarlabel,radarclr,fusedlabel,fusedclr);
    end

    fusedTrackLog{ind}  = fusedTracks;
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

    [trackAssignmentSummary(ind), truthAssignmentSummary(ind)] = assignmentMetrics(fusedTracks, truthsForMetrics);
    [assignedTrackIDs, assignedTruthIDs] = currentAssignment(assignmentMetrics);
    [posRMSE(ind), velRMSE(ind), posANEES(ind), velANEES(ind)] = errorMetrics(fusedTracks, assignedTrackIDs, truthsForMetrics, assignedTruthIDs);
    trackError{ind} = cumulativeTrackMetrics(errorMetrics);
    truthError{ind} = cumulativeTruthMetrics(errorMetrics);
end

wallClockTime = toc(a);
fprintf('  Sim loop done (%.1f s wall-clock).\n', wallClockTime);

summary = helperSummarizeAdsbDropout(analytics, 'IncludeOpenOutages', true);

% Quick look:
disp(summary.global);

% Per-target as a table:
disp(summary.perTargetTable);  % a MATLAB table with per-target metrics

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