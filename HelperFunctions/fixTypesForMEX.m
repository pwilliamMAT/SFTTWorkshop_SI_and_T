function trackStruct = helperFixTypesForMEX(trackStruct) %#ok<DEFNU>
if isempty(trackStruct)
    return;
end
for i = 1:length(trackStruct)
    if isfield(trackStruct(i), 'SourceIndex')
        srcIdx = trackStruct(i).SourceIndex; %#ok<NASGU>
    else
        warning('Track %d missing SourceIndex!', i);
    end
    if isfield(trackStruct(i), 'ObjectClassID')
        trackStruct(i).ObjectClassID = double(trackStruct(i).ObjectClassID);
    end
    if isfield(trackStruct(i), 'ObjectClassProbabilities')
        trackStruct(i).ObjectClassProbabilities = double(trackStruct(i).ObjectClassProbabilities);
    end
    if isfield(trackStruct(i), 'TrackLogicState')
        state = double(trackStruct(i).TrackLogicState);
        if any(state ~= 0 & state ~= 1)
            state = double(state >= 0.5);
        end
        trackStruct(i).TrackLogicState = state;
    end
    if isfield(trackStruct(i), 'ObjectAttributes')
        attrs = trackStruct(i).ObjectAttributes;
        if isempty(attrs) || ~isfield(attrs, 'Callsign') || ~isfield(attrs, 'Category')
            trackStruct(i).ObjectAttributes = struct(...
                'Callsign', repmat(' ', 1, 8), ...
                'Category', adsbCategory.No_Category_Information);
        end
    end
    if isfield(trackStruct(i), 'TrackLogic')
        trackStruct(i).TrackLogic = 'Integrated';
    end
end
end
