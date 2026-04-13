
function [tracker,activeRadarSpec,targetSpec] = helperCreateTracker(scenario)
passengerSpec = trackerTargetSpec("aerospace","aircraft","passenger");
%generalAviationSpec = trackerTargetSpec("aerospace","aircraft","general-aviation");
%helicopterSpec = trackerTargetSpec("aerospace","aircraft","helicopter");
%targetSpec = {passengerSpec,generalAviationSpec,helicopterSpec};
targetSpec = {passengerSpec}; % Omit other target specs, faster simulation with only one
activeRadarSpec = trackerSensorSpec("aerospace","radar","monostatic");


activeRadarSpec.MaxNumLooksPerUpdate = ceil(75*2/1.4); % Defines the number of looks in two seconds
activeRadarSpec.MaxNumMeasurementsPerUpdate = 20;
activeRadarSpec.PlatformPosition = [0 0 0];
activeRadarSpec.PlatformOrientation = eye(3);

activeRadarSpec = helperSyncSensor2spec(scenario.Platforms{1}.Sensors{1},activeRadarSpec);

tracker = multiSensorTargetTracker(targetSpec, activeRadarSpec, "jipda");
tracker.ConfirmationExistenceProbability = 0.95;
release(tracker);
end
