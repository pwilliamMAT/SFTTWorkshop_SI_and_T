function activeRadarData = helperRecordMonostaticData(sensorMountingAngles, detections, cfgs)
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