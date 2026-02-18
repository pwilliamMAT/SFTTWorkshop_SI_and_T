function [tracks] = trackingAlgorithm(dets,targetSpec,sensorSpec)
% trackingAlgorithm Summary of this function goes here

% Define the tracker as a persistent variable
persistent tracker

% Initialize the tracker on first call using isempty
if isempty(tracker)
    
    % Instantiate tracker
    tracker = multiSensorTargetTracker(targetSpec,sensorSpec,"jipda");
    tracker.ConfirmationExistenceProbability = 0.95; % Increased from default because clutter density is high

end

% Update the tracker every step using current detections and time stamp
tracks = tracker(dets{:});

end