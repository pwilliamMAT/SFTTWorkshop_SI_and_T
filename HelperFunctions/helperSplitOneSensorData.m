function data=helperSplitOneSensorData(sensorData,stopTime,trackerInterval)
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