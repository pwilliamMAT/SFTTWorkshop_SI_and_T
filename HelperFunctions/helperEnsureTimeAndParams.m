function tracksOut = helperEnsureTimeAndParams(tracksIn, time, params)
tracksOut = tracksIn;
for k = 1:numel(tracksOut)
    tracksOut(k).UpdateTime      = double(time);
    tracksOut(k).StateParameters = params;
end
end