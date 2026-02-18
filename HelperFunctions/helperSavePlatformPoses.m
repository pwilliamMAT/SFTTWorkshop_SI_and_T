function platformPoses = helperSavePlatformPoses(scenario,trackerInterval) %#ok<DEFNU>
endind = ceil(scenario.StopTime/trackerInterval);
samplePose = struct(PlatformID=1,ClassID=0,Position=zeros(1,3),Velocity=zeros(1,3),Acceleration=zeros(1,3),Orientation=eye(3),AngularVelocity=zeros(1,3));
platformPoses = repmat(samplePose,numel(scenario.Platforms),endind);
sampleTimes = trackerInterval*(1:endind);

for p = 1:numel(scenario.Platforms)
    thisPlatform = scenario.Platforms{p};
    trajectory = thisPlatform.Trajectory;
    if isa(trajectory,"waypointTrajectory")
        [position,orientation,velocity,acceleration,angularVelocity] = lookupPose(trajectory,sampleTimes);
        orientation = rotmat(orientation,"frame");
        for i = 1:endind
            platformPoses(p,i) = struct(PlatformID=thisPlatform.PlatformID,ClassID=thisPlatform.ClassID, ...
                Position=position(i,:),Velocity=velocity(i,:),Acceleration=acceleration(i,:), ...
                Orientation=orientation(:,:,i),AngularVelocity=angularVelocity(i,:));
        end
    else
        platformPoses(p,:) = repmat(struct(PlatformID=thisPlatform.PlatformID,ClassID=thisPlatform.ClassID, ...
            Position=thisPlatform.Position,Velocity=zeros(1,3),Acceleration=zeros(1,3),Orientation=thisPlatform.Orientation, ...
            AngularVelocity=zeros(1,3)),1,endind);
    end
end
end