function radarTrack = helperEcef2nedTrack(centralTrack)
mapOrigin = [42.39423231362 -70.95934958874 0];
radarTrack = objectTrack('State',zeros(6,1),...
    'StateCovariance',eye(6));
radarTrack = helperSyncTrack(radarTrack,centralTrack);

centerState = centralTrack.State;
[N,E,D] = ecef2ned(centerState(1),centerState(3),centerState(5),mapOrigin(1),mapOrigin(2),mapOrigin(3),wgs84Ellipsoid);
[vN,vE,vD] = ecef2nedv(centerState(2),centerState(4),centerState(6),mapOrigin(1),mapOrigin(2));
radarState = [N, vN, E, vE, D, vD];

centerStateCov = centralTrack.StateCovariance;
R = dcmecef2ned(mapOrigin(1), mapOrigin(2));
permute_idx = [1, 3, 5, 2, 4, 6];
unpermute_idx = [1, 4, 2, 5, 3, 6];
P_perm = centerStateCov(permute_idx, permute_idx);
R6 = blkdiag(R, R);
P_rot = R6 * P_perm * R6';
radarStateCov = P_rot(unpermute_idx, unpermute_idx);

radarTrack.State = radarState;
radarTrack.StateCovariance = radarStateCov;
end