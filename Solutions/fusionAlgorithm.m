function fusedTracks = fusionAlgorithm(tracks, time) %#codegen
% tracks: struct array (codegen-friendly, not objectTrack)
% time:   scalar double

persistent fuser
if isempty(fuser)
    % Use ONE pair of transform handles for ALL sources
    % (homogeneous handles; per-source dispatch happens inside these functions)
    % NOTE: Do not assign to variables named like the functions; pass the handles directly.
    % MUST match main script configuration: Radar updates, ADSB initializes
    radarCfg = fuserSourceConfiguration( ...
        'SourceIndex',1, ...
        'IsInitializingCentralTracks',false, ...
        'CentralToLocalTransformFcn', @central2local, ...
        'LocalToCentralTransformFcn', @local2central);

    adsbCfg  = fuserSourceConfiguration( ...
        'SourceIndex',2, ...
        'IsInitializingCentralTracks',true, ...
        'CentralToLocalTransformFcn', @central2local, ...
        'LocalToCentralTransformFcn', @local2central);


    % Name-value pairs are literals (compile-time constants) when written like this
    fuser = trackFuser( ...
        'FuserIndex',            3, ...
        'MaxNumSources',         2, ...
        'ProcessNoise', blkdiag(33.3333,33.3333,0.3333),...
        'SourceConfigurations',  {radarCfg; adsbCfg}, ...
        'AssignmentThreshold',   [100 250], ...
        'StateFusion',           'Intersection', ...
        'StateFusionParameters', 'trace', ...
        'ConfirmationThreshold', [2 3], ...
        'DeletionThreshold',     [2 2]);
end

% Step the fuser
fusedTracks = fuser(tracks, time);
end



% HELPER FUNCTIONS
% ---- Wrappers (homogeneous handles) ----
function centralTrack = local2central(localTrack)
% Pre-define output - force 'Integrated' for codegen consistency
state = zeros(6,1);
stateCov = eye(6);
centralTrack = objectTrack('State',state,'StateCovariance',stateCov,'TrackLogic','Integrated');

srcIdx = localTrack.SourceIndex;
if srcIdx == 1 % Radar
    centralTrack = Ned2ecefTrack(localTrack);
elseif srcIdx == 2 % ADSB
    centralTrack = Ecef2nedTrack(localTrack);
end
end

function localTrack = central2local(centralTrack)
% Pre-define output with longest TrackLogic to force codegen consistency
state = zeros(6,1);
stateCov = eye(6);
localTrack = objectTrack('State',state,'StateCovariance',stateCov,'TrackLogic','Integrated');

srcIdx = centralTrack.SourceIndex;
if srcIdx == 1 % Radar
    localTrack = Ecef2nedTrack(centralTrack);
elseif srcIdx == 2 % ADSB
    localTrack = Ned2ecefTrack(centralTrack);
end
end


function centralTrack = Ned2ecefTrack(radarTrack)
mapOrigin = [42.39423231362 -70.95934958874 0];
% Initialize a track of the correct state size
% Force 'Integrated' for codegen consistency
centralTrack = objectTrack('State',zeros(6,1),...
    'StateCovariance',eye(6),'TrackLogic','Integrated');

% Sync properties of radarTrack except State and StateCovariance with
% radarTrack See syncTrack defined below.
centralTrack = syncTrack(centralTrack,radarTrack);

% Convert NED state to ECEF state
radarState = radarTrack.State;
[X,Y,Z] = ned2ecef(radarState(1),radarState(3),radarState(5),mapOrigin(1),mapOrigin(2),mapOrigin(3),wgs84Ellipsoid); 

% Rotation matrix ECEF->NED (codegen-compatible)
R = ecef2nedRotmat(mapOrigin(1), mapOrigin(2)); % Note dcmecef2ned not supported for codegen

% NED->ECEF velocity: v_ecef = R' * v_ned
velos = R' * [radarState(2);radarState(4);radarState(6)];
centerState = [X, velos(1), Y, velos(2), Z, velos(3)];

% Convert NED state COVARIANCE to ECEF state COVARIANCE
radarStateCov = radarTrack.StateCovariance;

% Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz]
permute_idx = [1, 3, 5, 2, 4, 6];
unpermute_idx = [1, 4, 2, 5, 3, 6];

% Permute covariance matrix to group pos/vel
P_perm = radarStateCov(permute_idx, permute_idx);

% NED->ECEF covariance: P_ecef = R' * P_ned * R
R6 = blkdiag(R', R');
P_rot = R6 * P_perm * R6';

% Unpermute back to original interleaved format
centerStateCov = P_rot(unpermute_idx, unpermute_idx);

% Set state and covariance of central track
centralTrack.State = centerState;
centralTrack.StateCovariance = centerStateCov;
end

function radarTrack = Ecef2nedTrack(centralTrack)
% A function to transform a track in the central state-space to a track in
% the radar state-space.
mapOrigin = [42.39423231362 -70.95934958874 0];
% Initialize a track of the correct state size
% Force 'Integrated' for codegen consistency
radarTrack = objectTrack('State',zeros(6,1),...
    'StateCovariance',eye(6),'TrackLogic','Integrated');

% Sync properties of radarTrack except State and StateCovariance with
% radarTrack See syncTrack defined below.
radarTrack = syncTrack(radarTrack,centralTrack);

% Convert ECEF state to NED
centerState = centralTrack.State;
[N,E,D] = ecef2ned(centerState(1),centerState(3),centerState(5),mapOrigin(1),mapOrigin(2),mapOrigin(3),wgs84Ellipsoid );

% ECEF->NED velocity via built-in
[vN,vE,vD] = ecef2nedv(centerState(2),centerState(4),centerState(6),mapOrigin(1),mapOrigin(2));
radarState = [N, vN, E, vE, D, vD];

% Convert ECEF state COVARIANCE to NED state COVARIANCE
centerStateCov = centralTrack.StateCovariance;

% Rotation matrix ECEF->NED (codegen-compatible)
R = ecef2nedRotmat(mapOrigin(1), mapOrigin(2)); % Note dcmecef2ned not supported for codegen

% Permutation indices: from [x vx y vy z vz] to [x y z vx vy vz]
permute_idx = [1, 3, 5, 2, 4, 6];
unpermute_idx = [1, 4, 2, 5, 3, 6];

% Permute covariance matrix to group pos/vel
P_perm = centerStateCov(permute_idx, permute_idx);

% ECEF->NED covariance: P_ned = R * P_ecef * R'
R6 = blkdiag(R, R);
P_rot = R6 * P_perm * R6';

% Unpermute back to original interleaved format
radarStateCov = P_rot(unpermute_idx, unpermute_idx);

% Set state and covariance of radar track
radarTrack.State = radarState;
radarTrack.StateCovariance = radarStateCov;
end


% Updated to comply with codegen (different from local code only in that
% we're casting values for consistency)
function dst = syncTrack(dst, src)
% Copy scalars (cast to double/logical for objectTrack assignments)
dst.TrackID         = double(src.TrackID);
dst.BranchID        = double(src.BranchID);
dst.SourceIndex     = double(src.SourceIndex);
dst.UpdateTime      = double(src.UpdateTime);
dst.Age             = double(src.Age);
dst.ObjectClassID   = double(src.ObjectClassID);
dst.ObjectClassProbabilities = double(src.ObjectClassProbabilities);
%dst.TrackLogic = src.TrackLogic;
% Skip TrackLogicState - fuser manages this internally, and copying causes codegen size issues
%dst.TrackLogicState = double(src.TrackLogicState);
dst.IsConfirmed     = logical(src.IsConfirmed);
dst.IsCoasted       = logical(src.IsCoasted);
dst.IsSelfReported  = logical(src.IsSelfReported);

% States / covariance
dst.State           = double(src.State);
dst.StateCovariance = double(src.StateCovariance);

% Parameters and Attributes
dst.StateParameters = struct();
dst.ObjectAttributes = struct();
end

function R = ecef2nedRotmat(lat_deg, lon_deg)
%ecef2nedRotmat  3x3 ECEF-to-NED rotation matrix (codegen-compatible).
%   R = ecef2nedRotmat(lat_deg, lon_deg) returns the same matrix as
%   dcmecef2ned(lat_deg, lon_deg) from Aerospace Toolbox, but uses only
%   basic trig — fully supported by MATLAB Coder.
%
%   Usage:  v_ned  = R  * v_ecef     (ECEF → NED)
%           v_ecef = R' * v_ned      (NED  → ECEF)
    lat = deg2rad(lat_deg);
    lon = deg2rad(lon_deg);
    sLat = sin(lat);  cLat = cos(lat);
    sLon = sin(lon);  cLon = cos(lon);
    R = [-sLat*cLon, -sLat*sLon,  cLat;
         -sLon,       cLon,       0;
         -cLat*cLon, -cLat*sLon, -sLat];
end
