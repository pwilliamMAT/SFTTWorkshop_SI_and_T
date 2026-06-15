%[text] # System Integration and Test Simulation
%[text] Configure a simulation that uses the following:
%[text] - Truth Locations - Field-collected ADSB data recorded with an RTL-SDR
%[text] - Detections and Sensor-based Tracks:
%[text] - Active radar (fusionRadarSensor)
%[text] - ADSB data (gpsSensor and ADSBReceiver)
%[text] - TrackFuser for C2 level tracks
%[text] - OSPA Metric, trackAssignmentMetrics, and errorMetrics for track accuracy analysis \
%[text:tableOfContents]{"heading":"Table of Contents"}
close all force; clear; addpath('../HelperFunctions/')
%[text] ## Setup Scenario
%[text] Import truth location of aircraft and convert to trajectories for simulation
mapOrigin = [42.39423231362 -70.95934958874 0]; % Hard-coded this into radar2central and central2radar
load("TruthData.mat")
%[text] Scenario Creation includes configuring the Active Radar Sensor
simStopTime = 60; % s
scenario = helperCreateScenario(tuningData,mapOrigin,simStopTime);
mapViewer = trackingGlobeViewer('ReferenceLocation',mapOrigin,'Basemap','streets-dark');
campos(mapViewer, mapOrigin + [0 -0.4 1e5]);
drawnow;
plotScenario(mapViewer,scenario);
%snapshot(mapViewer); % Only supported in MATLAB Desktop
%%
%[text] ## **Configure Tracker**
%[text] ### Specify Aircraft Types
%[text] The first step in defining the tracker requires specifying the types of objects you want to track. In this example, these objects are passenger aircraft, general aviation aircraft, and helicopters. Use the [`trackerTargetSpec`](docid:fusion_ref#mw_d12c6e07-1099-4b7b-b09f-3be62d29245e) function to create a target specification for a passenger aircraft and observe the properties of the specification.
%[text] ## **Step 1 - Create a Passenger Aircraft Target Specification using the trackerTargetSpec**
%[text] **Directions:**
%[text] Below, please create a variable 'passengerSpec' that holds the Passenger Aircraft Target Specification generated using 'trackerTargetSpec'. Use the "aerospace", "aircraft", and "passenger" inputs to specify the target specification.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
doc trackerTargetSpec % Opens documentation page for trackerTargetSpec
% WRITE YOUR CODE BELOW THIS LINE:

% Define passengerSpec

%[text] Display the Target Specification:
disp(passengerSpec) %[output:7ab087c2]
activeRadarSpec = trackerSensorSpec("aerospace","radar","monostatic");
targetSpec = {passengerSpec};

towerPosition = mapOrigin;
activeRadarSpec.MaxNumLooksPerUpdate = ceil(75*2/1.4); % Defines the number of looks in two seconds
activeRadarSpec.MaxNumMeasurementsPerUpdate = 20;
activeRadarSpec.PlatformPosition = [0 0 0];
activeRadarSpec.PlatformOrientation = eye(3);

activeRadarSpec = helperSyncSensor2spec(scenario.Platforms{1}.Sensors{1},activeRadarSpec);
%[text] ### Configure Tracker to Use Target and Sensor Specifications
%[text] The last step in specifying the tracker is to configure it to use the heterogeneous target and sensor specifications you have defined.
tracker = multiSensorTargetTracker({passengerSpec},activeRadarSpec,"jipda");
tracker.ConfirmationExistenceProbability = 0.95; % Increased from default because clutter density is high
release(tracker);
%%
%[text] ## Record Detection Data 
%[text] By loading the data that was recorded during a previous run, we speed up simulation. If further adjustments are made to scenario, "sensorData = helperRecordSensorData(scenario);" should be run to reflect those changes.
load('SensorData.mat');
%[text] Reset Scenario, Tracker, Viewer, and Transponders - Running helperRecordSensorData above leaves the simulation at the end time.
clear(mapViewer);
restart(scenario);
release(tracker);
%[text] %[text:anchor:TMP_8b95] Split dataset for easier parsing within simulation below.
trackerUpdateInterval = 1; % seconds %[control:slider:23f3]{"position":[25,26]}
[activeRadarData] = helperSplitSensorData(sensorData,scenario.StopTime,trackerUpdateInterval);
savedPoses = helperSavePlatformPoses(scenario,trackerUpdateInterval);
%%
%[text] ## Configure Simulation
a = tic; % Time the simulation
detLog = [];
detBuffer = {};

% Preallocate variables for logging data for simulation:
truePoseGPSAll = cell(1, numel(activeRadarData));
radarTrackLog = cell(1, numel(activeRadarData));
trackError = cell(1, numel(activeRadarData));
truthError = cell(1, numel(activeRadarData));
plotPlatform(mapViewer,[scenario.Platforms{3:end}],TrajectoryMode="Full");
%[text] Configure track labels and colors
radarlabel = "  Radar";
radarclr = [255 255 17]/255;
%%
%[text] ## Instantiate Error Metrics and Assignment Metrics
% Define errorMetrics
errorMetrics = trackErrorMetrics;

% Define assignmentMetrics
assignmentMetrics = trackAssignmentMetrics(...
    'AssignmentDistance','posabserr',...
    'DivergenceDistance','posabserr',...
    'DivergenceThreshold',50,...
    'AssignmentThreshold',50);
%[text] ## **Step 2- Instantiate the** trackOSPAMetrics **to estimate the tracks' cost compared to truth**
%[text] **Directions:**
%[text] Below, please create a variable 'ospa' that holds the track OSPA metrics generated using 'trackOSPAMetrics'
%[text] Using Name-Value pairs to configure the trackOSPAMetrics, set the 'CutoffDistance' to 500 and the 'Distance' to 'posabserr' for position absolute error
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
doc trackOSPAMetric % Open documentation page for trackOSPAMetric
% WRITE YOUR CODE BELOW THIS LINE:

%%
%[text] ## Run Scenario
%[text] This takes about 2 minutes to run.
for ind = 1:numel(activeRadarData)
    advance(scenario);

    % Plot Coverage of Radar
    covcon = coverageConfig(scenario);
    plotCoverage(mapViewer,covcon,"ECEF") %,Color=[0 1 0]

    % Gather true positions
    for i = 1:numel(scenario.Platforms)
        poseStruct = pose(scenario.Platforms{i},'CoordinateSystem','Geodetic');
        poseStruct.PlatformID = scenario.Platforms{i}.PlatformID;
        truePoseGPS(i) = poseStruct;
        truePoseNED(i) = poseStruct;
        truePoseNED(i).Position = lla2ned(truePoseGPS(i).Position,mapOrigin,'ellipsoid');
    end
    truePoseGPSAll{ind} = truePoseGPS;
    truePoseNEDAll{ind} = truePoseNED;
    truePosePlot{ind} = platformPoses(scenario);

    % --- Plot Truth Locations ---
    plotPlatform(mapViewer, truePosePlot{ind}, 'ECEF');

    % --- Plot Detections ---
    helperPlotActiveRadarData(mapViewer, activeRadarSpec, activeRadarData(ind));
    time = activeRadarData(ind).DetectionTime(1);

    % Instantiate variables for plotting - does not effect tracker
    % continuity
    radarTracks = objectTrack.empty;

    % Radar Processing

    % Use MEX version for speedup - outputs struct due to
    % codegeneration requirements
    radarTracks = trackingAlgorithm_mex({activeRadarData(ind)},targetSpec,activeRadarSpec);

    % Convert Struct output to objectTrack
    radarTracks = helperConvertToObjectTrack(radarTracks);

    % Plotting
    labels = repmat(radarlabel, 1, numel(radarTracks));
    colors = repmat(radarclr, numel(radarTracks), 1);

    if ~isempty(radarTracks)
        plotTrack(mapViewer, radarTracks, ...
            'LabelStyle','Custom', ...
            "CustomLabel", repmat(radarlabel, 1, numel(radarTracks)), ...
            'Color', repmat(radarclr, numel(radarTracks), 1), ...
            'LineWidth', 3);
    end

    % Logging
    radarTrackLog{ind} = radarTracks;

    % Metrics
    [trackAssignmentSummary(ind), truthAssignmentSummary(ind)] = assignmentMetrics(radarTracks, truePoseNED);
    [assignedTrackIDs, assignedTruthIDs] = currentAssignment(assignmentMetrics);
    [posRMSE(ind), velRMSE(ind), posANEES(ind), velANEES(ind)] = errorMetrics(radarTracks, assignedTrackIDs, truePoseNED, assignedTruthIDs);
    trackError{ind} = cumulativeTrackMetrics(errorMetrics);
    truthError{ind} = cumulativeTruthMetrics(errorMetrics);
end
toc(a)
%[text] Take Snapshot of Viewer
% snapshot(mapViewer); % Only supported in MATLAB Desktop
%%
%[text] ## Collect and Visualize Metrics
%[text] ### Compute OSPA
% Filter truths to moving targets only (drop first 2 platforms = sensor towers)
truthsFilteredNEDAll = cell(size(truePoseNEDAll));
for i = 1:numel(truePoseNEDAll)
    tp = truePoseNEDAll{i};
    if numel(tp) > 2
        tp = tp(3:end);
    else
        tp = tp([]);
    end
    truthsFilteredNEDAll{i} = tp;
end

% Radar OSPA
N_ospa = numel(radarTrackLog);
radarospa = nan(N_ospa,1); radarLoc = nan(N_ospa,1); radarCard = nan(N_ospa,1);
for i = 1:N_ospa
    tracks = radarTrackLog{i};
    truths = truthsFilteredNEDAll{i};
    if isempty(tracks) && isempty(truths), continue; end
    [radarospa(i), radarLoc(i), radarCard(i)] = ospa(tracks, truths);
end
%%
%[text] ### Plot OSPA
radarclr = [255 255 17]/255;

figure

plot((0:(N_ospa-1))/60, radarospa, "Color",radarclr);
l=legend('Primary (Radar)');
l.Color = [0.1 0.1 0.1];
l.TextColor = [1 1 1];
xlabel('Time (min)')
ylabel('OSPA')
ax = gca;
grid on; box on;
ax.Color = [0.1 0.1 0.1];
ax.GridColor = [1 1 1];
%%
%[text] ### Plot RMS Values
figure;
timeVec = 1:length(posRMSE);
subplot(2,2,1)
scatter(timeVec,posRMSE,'filled');
xlabel('Time step');
ylabel('RMS Error in Position (m)');
grid('on');

subplot(2,2,2)
scatter(timeVec,velRMSE,'filled');
xlabel('Time step');
ylabel('RMS Error in Velocity (m/s)');
grid('on');

subplot(2,2,3)
scatter(timeVec,posANEES,'filled');
xlabel('Time step');
ylabel('Average Normalized Error in Position');
grid('on');

subplot(2,2,4)
scatter(timeVec,velANEES,'filled');
xlabel('Time step');
ylabel('Average Normalized Error in Velocity');
grid('on');

sgtitle('Cumulative errors for all radar tracks','FontWeight','bold');
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":10.5}
%---
%[control:slider:23f3]
%   data: {"defaultValue":1,"label":"trackerUpdateInterval","max":5,"min":0.1,"run":"Section","runOn":"ValueChanging","step":0.1}
%---
%[output:7ab087c2]
%   data: {"dataType":"text","outputData":{"text":"  <a href=\"matlab:helpPopup('fusion.tracker.targetspecs.PassengerAircraft')\" style=\"font-weight:bold\">PassengerAircraft<\/a> with properties:\n\n           MaxHorizontalSpeed: 250    m\/s \n             MaxVerticalSpeed: 20     m\/s \n    MaxHorizontalAcceleration: 10     m\/s²\n      MaxVerticalAcceleration: 1      m\/s²\n\n","truncated":false}}
%---
