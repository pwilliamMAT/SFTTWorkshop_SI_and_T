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