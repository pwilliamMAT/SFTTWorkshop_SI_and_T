function plotActiveRadarData(mapViewer, radarSpec, radarData, args)

arguments
    mapViewer
    radarSpec
    radarData
    args.PlotCoverage = false;
end

clrs = lines(7);

if args.PlotCoverage
    covActive = coverageConfig(radarSpec, radarData);
    plotCoverage(mapViewer, covActive, 'Color', clrs(covActive.Index,:));
end

radarDets = radarDataToObjectDetection(radarSpec, radarData);


% UGLY WORKAROUND TO FORCE GLOBE VIEWER TO DELETE PAST DETECTIONS
if isempty(radarDets)
    radarDets = {objectDetection(0,[0;0;1e10;0],'SensorIndex',1,'MeasurementParameters',struct('Frame','spherical'))};
end

plotDetection(mapViewer, radarDets, 'NED', 'Color',clrs(1,:));

end


function detections = radarDataToObjectDetection(radarSpec, radarData)
n = size(radarData.Azimuth,2);
mp = struct('Frame','spherical',...
    'OriginPosition',zeros(3,1),...
    'OriginVelocity',zeros(3,1),...
    'Orientation',eye(3),...
    'HasAzimuth',true,...
    'HasElevation',true,...
    'HasRange',true,...
    'HasVelocity',true,...
    'IsParentToChild',true);
mp = repmat(mp,2,1);

sampleDet = objectDetection(0,zeros(4,1),'MeasurementParameters',mp,'SensorIndex',1);
detections = repmat({sampleDet},n,1);
for i = 1:n
    detections{i}.Measurement = [radarData.Azimuth(i);radarData.Elevation(i);radarData.Range(i);radarData.RangeRate(i)];
    detections{i}.MeasurementNoise = blkdiag(radarData.AzimuthAccuracy(i)^2,radarData.ElevationAccuracy(i)^2,radarData.RangeAccuracy(i)^2,radarData.RangeRateAccuracy(i)^2);
    detections{i}.MeasurementParameters(1).OriginPosition = radarSpec.MountingLocation(:);
    detections{i}.MeasurementParameters(2).OriginPosition = radarSpec.PlatformPosition(:); 
    
    detections{i}.MeasurementParameters(1).OriginVelocity = zeros(3,1);
    detections{i}.MeasurementParameters(2).OriginVelocity = zeros(3,1);

    [~,scanIdx] = min(abs(radarData.LookTime - radarData.DetectionTime(i)));
    lookAng = [radarData.LookAzimuth(scanIdx);radarData.LookElevation(scanIdx)];
    ypr = zeros(1,3,'like',lookAng);
    ypr(1:2) = lookAng;
    ypr(2) = -ypr(2); % Elevation is opposite the sign of pitch
    lookRot = fusion.internal.frames.ypr2rotmat(ypr,'degrees');

    detections{i}.MeasurementParameters(1).Orientation = lookRot*fusion.internal.frames.ypr2rotmat(radarSpec.MountingAngles,'degrees');
    detections{i}.MeasurementParameters(2).Orientation = radarSpec.PlatformOrientation;

    detections{i}.MeasurementParameters(2).Frame = 'rectangular';
end

end


function cvg = coverageConfig(spec, data)

cvg = struct('Index',1,...
    'LookAngle',zeros(2,1),...
    'FieldOfView',zeros(2,1),...
    'ScanLimits',[0 0],...
    'Range',1e5,...
    'Position',[0 0 0],...
    'Orientation',quaternion.ones(1,1));
pos = zeros(3,1);
sensorRot = rotmat(quaternion(spec.MountingAngles,'eulerd','ZYX','frame'),'frame');
sensorPos = spec.MountingLocation;
platformRot = spec.PlatformOrientation;
platformPos = spec.PlatformPosition;
pos = fusion.tracker.internal.utils.transformToParent(pos, sensorPos(:), sensorRot);
pos = fusion.tracker.internal.utils.transformToParent(pos, platformPos(:), platformRot);
rot = quaternion(platformRot*sensorRot,'rotmat','frame');
cvg.Position = pos';
cvg.Orientation = rot;
az = data.LookAzimuth;
fovAz = abs(wrapTo180(az(end) - az(1)));
el = data.LookElevation;
fovEl = abs(wrapTo180(el(end) - el(1)));
fov = [fovAz;fovEl] + spec.FieldOfView(:);
idx = max(1,floor(numel(az)/2));
cvg.LookAngle = [wrapTo180(az(idx));wrapTo180(el(idx))];
cvg.FieldOfView = fov;

end