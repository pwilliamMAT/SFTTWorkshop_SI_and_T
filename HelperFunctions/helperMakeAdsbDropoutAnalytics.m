function analytics = helperMakeAdsbDropoutAnalytics(Ts)
% makeAdsbDropoutAnalytics  Create analytics state for ADS-B dropout.
% Ts : sample time in seconds (default 1 sec)
    if nargin < 1 || isempty(Ts), Ts = 1; end

    analytics.Ts = Ts;
    analytics.perTarget = containers.Map('KeyType','char','ValueType','any');

    analytics.global.expected = 0;
    analytics.global.kept     = 0;
    analytics.global.dropped  = 0;

    analytics.started = false;
end