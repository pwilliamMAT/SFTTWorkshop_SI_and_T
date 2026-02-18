function varargout=helperSplitSensorData(sensorData,stopTime,trackerInterval)
varargout = cell(1,nargout);
if iscell(sensorData)
    varargout = cellfun(@(s) helperSplitOneSensorData(s,stopTime,trackerInterval), sensorData, UniformOutput=false);
else
    varargout{1} = helperSplitOneSensorData(sensorData,stopTime,trackerInterval);
end
end