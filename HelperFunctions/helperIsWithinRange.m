function flag = helperIsWithinRange(stationsLLA, posStruct, range)
for i = 3:length(posStruct)
    pos = posStruct(i).Position;
    cartDistance = fusion.internal.frames.lla2ecef(stationsLLA) - fusion.internal.frames.lla2ecef(pos);
    flag(i-2) = any(vecnorm(cartDistance,2,2) < range);
end
end