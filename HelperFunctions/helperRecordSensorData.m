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