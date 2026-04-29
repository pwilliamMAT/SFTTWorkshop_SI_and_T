% Setup Scenario
mapOrigin = [42.39423231362 -70.95934958874 0]; % Hard-coded this into radar2central and central2radar
load("TrackingScenarioTruth.mat")
scenario = helperCreateScenario(tuningData,mapOrigin);

%  Configure Target Specifications
passengerSpec = trackerTargetSpec("aerospace","aircraft","passenger");
targetSpec = {passengerSpec}; % Switch to single target spec, runs faster with passenger only
%generalAviationSpec = trackerTargetSpec("aerospace","aircraft","general-aviation");
%helicopterSpec = trackerTargetSpec("aerospace","aircraft","helicopter");
%targetSpec = {passengerSpec,generalAviationSpec,helicopterSpec};


% Declare fixed-size cell array for targetSpec
%targetSpecType = coder.typeof({passengerSpec, generalAviationSpec, helicopterSpec}, [1 3], [false false]);
targetSpecType = coder.typeof({passengerSpec}, [1 1], [false false]);


% Configure Sensor Specification
activeRadarSpec = trackerSensorSpec("aerospace","radar","monostatic");
activeRadarSpec.MaxNumLooksPerUpdate = ceil(75*2/1.4); % Defines the number of looks in two seconds
activeRadarSpec.MaxNumMeasurementsPerUpdate = 20;
activeRadarSpec.PlatformPosition = [0 0 0];
activeRadarSpec.PlatformOrientation = eye(3);
activeRadarSpec = helperSyncSensor2spec(scenario.Platforms{1}.Sensors{1},activeRadarSpec);

% Lock sensorSpec type
activeRadarSpec.MaxNumLooksPerUpdate = 108; % fixed
activeRadarSpec.MaxNumMeasurementsPerUpdate = 20; % fixed
sensorSpecType = coder.typeof(activeRadarSpec);

% Detections formatting for codegen:
maxDetections = 50;  % upper bound for per-step detections
maxLooks = 100;      % upper bound for LookTime array

% Create a prototype struct with max sizes
proto = struct( ...
    'LookTime', coder.typeof(0, [1 maxLooks], [false true]), ...
    'LookAzimuth', coder.typeof(0, [1 maxLooks], [false true]), ...
    'LookElevation', coder.typeof(0, [1 maxLooks], [false true]), ...
    'DetectionTime', coder.typeof(0, [1 maxDetections], [false true]), ...
    'Azimuth', coder.typeof(0, [1 maxDetections], [false true]), ...
    'Elevation', coder.typeof(0, [1 maxDetections], [false true]), ...
    'Range', coder.typeof(0, [1 maxDetections], [false true]), ...
    'RangeRate', coder.typeof(0, [1 maxDetections], [false true]), ...
    'AzimuthAccuracy', coder.typeof(0, [1 maxDetections], [false true]), ...
    'ElevationAccuracy', coder.typeof(0, [1 maxDetections], [false true]), ...
    'RangeAccuracy', coder.typeof(0, [1 maxDetections], [false true]), ...
    'RangeRateAccuracy', coder.typeof(0, [1 maxDetections], [false true]) ...
);


% Declare type with bounds
detectionType = coder.typeof(proto, [1 1], [false false]);
detectionCellType = coder.typeof({proto}, [1 1], [false false]);

% Generate MEX of ToT tracker
codegen -report trackingAlgorithm.m -args {detectionCellType, targetSpecType, sensorSpecType}


% Generate MEX of ToT tracker
%codegen -report trackingAlgorithm.m -args {singleStepDets,targetSpec,activeRadarSpec}


% HELPER FUNCTIONS
function spec = helperSyncSensor2spec(sensor,spec)
if isa(sensor,'fusionRadarSensor') && strcmpi(sensor.DetectionMode,'monostatic')
    spec = syncRadar2spec(sensor,spec);
elseif isa(sensor,'fusionRadarSensor') && strcmpi(sensor.DetectionMode,'esm')
    spec = syncESM2spec(sensor,spec);
end
end

function spec = syncRadar2spec(sensor,spec)
spec.MountingLocation = sensor.MountingLocation;
spec.MountingAngles = sensor.MountingAngles;
spec.HasElevation = sensor.HasElevation;
spec.HasRangeRate = sensor.HasRangeRate;
spec.FieldOfView = sensor.FieldOfView;
spec.RangeLimits = sensor.RangeLimits;
spec.RangeRateLimits = sensor.RangeRateLimits;
spec.AzimuthResolution = sensor.AzimuthResolution;
spec.RangeResolution = sensor.RangeResolution;
spec.ElevationResolution = sensor.ElevationResolution;
spec.RangeRateResolution = sensor.RangeRateResolution;
spec.DetectionProbability = sensor.DetectionProbability;
spec.FalseAlarmRate = sensor.FalseAlarmRate;
end

function scenario = helperCreateScenario(tuningData,mapOrigin)
scenarioDuration = 200; % s
scenario = trackingScenario(UpdateRate=1,StopTime=scenarioDuration,IsEarthCentered=true);
radarTower = platform(scenario, Position=mapOrigin);

rpm = 20;
beamwidthAz = 360; % degrees
fov = [beamwidthAz; 15]; % narrower azimuth FoV
updaterate = 1; % Hz, or match scan revisit rate
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
    

% Mount radar at the top of the tower
radarTower.Sensors = activeRadar;

% Set mechanical elevation scan to begin at 2 degrees above the horizon
elFov = fov(2);
%tilt = 2; % deg
%activeRadar.MechanicalElevationLimits = [-fov(2) 0]-tilt; % deg
activeRadar.FieldOfView(2) = elFov+1e-3;

% Place ADSB Tower
%applehillPos = [-89.5298 2.0521e+03 0]; % NED - isEarthCentered=false
applehillPos = [42.300498, -71.349157 0]; % LLA- isEarthCentered=true
ADSBTower = platform(scenario, Position=applehillPos); %[-5000 -3000 -50]

% Create Platforms for each Aircraft in TuningData
for i = 1:numel(tuningData)
       
    % Geo Trajectory - isEarthCentered = true
    Pos = tuningData{i}.Position;
    time = seconds(tuningData{i}.Time);
    
    % Find indices where time is within the scene duration
    validIdx = time <= scenarioDuration;

    % Truncate time and position vectors
    truncatedTime = time(validIdx);
    truncatedPos = Pos(validIdx, :);
    
    % Generate Trajectory 
    traj = geoTrajectory(truncatedPos,truncatedTime);

    % Create Platform that follows above trajectory 
    platform(scenario,Trajectory=traj);
end
end