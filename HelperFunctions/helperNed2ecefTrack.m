function centralTrack = helperNed2ecefTrack(radarTrack)
mapOrigin = [42.39423231362 -70.95934958874 0];
centralTrack = objectTrack('State',zeros(6,1),...
    'StateCovariance',eye(6));
centralTrack = syncTrack(centralTrack,radarTrack);

radarState = radarTrack.State;
[X,Y,Z] = ned2ecef(radarState(1),radarState(3),radarState(5),mapOrigin(1),mapOrigin(2),mapOrigin(3),wgs84Ellipsoid); 
R = dcmecef2ned(mapOrigin(1), mapOrigin(2));
velos = R' * [radarState(2);radarState(4);radarState(6)];
centerState = [X, velos(1), Y, velos(2), Z, velos(3)];

radarStateCov = radarTrack.StateCovariance;
permute_idx = [1, 3, 5, 2, 4, 6];
unpermute_idx = [1, 4, 2, 5, 3, 6];
P_perm = radarStateCov(permute_idx, permute_idx);
R6 = blkdiag(R', R');
P_rot = R6 * P_perm * R6';
centerStateCov = P_rot(unpermute_idx, unpermute_idx);

centralTrack.State = centerState;
centralTrack.StateCovariance = centerStateCov;
end