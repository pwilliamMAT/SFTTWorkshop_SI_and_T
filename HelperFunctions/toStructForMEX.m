function s = toStructForMEX(tracks)
s = toStruct(tracks);
for k = 1:numel(s)
    s(k).TrackID     = uint32(s(k).TrackID);
    s(k).BranchID    = uint32(s(k).BranchID);
    s(k).SourceIndex = uint32(s(k).SourceIndex);
    s(k).Age         = uint32(s(k).Age);
    s(k).UpdateTime  = double(s(k).UpdateTime);
    s(k).State       = double(s(k).State);
    s(k).StateCovariance = double(s(k).StateCovariance);
    s(k).ObjectClassID   = double(s(k).ObjectClassID);
    s(k).ObjectClassProbabilities = double(s(k).ObjectClassProbabilities);
    s(k).TrackLogicState = double(s(k).TrackLogicState);
    s(k).IsConfirmed    = logical(s(k).IsConfirmed);
    s(k).IsCoasted      = logical(s(k).IsCoasted);
    s(k).IsSelfReported = logical(s(k).IsSelfReported);
    s(k).TrackLogic = 'Integrated';
    s(k).StateParameters = struct();
    attrs = s(k).ObjectAttributes;
    if isempty(attrs) || ~isfield(attrs, 'Callsign') || ~isfield(attrs, 'Category')
        s(k).ObjectAttributes = struct( ...
            'Callsign', repmat(' ', 1, 8), ...
            'Category', adsbCategory.No_Category_Information);
    end
end
end