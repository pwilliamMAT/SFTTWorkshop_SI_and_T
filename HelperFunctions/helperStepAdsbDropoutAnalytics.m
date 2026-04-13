function analytics = helperStepAdsbDropoutAnalytics(analytics, preMsgs, postMsgs, t)
% stepAdsbDropoutAnalytics  Update analytics using pre-/post-drop message sets.
%
% Inputs:
%   preMsgs  : Nx1 struct array (in-range BEFORE dropout)
%   postMsgs : Mx1 struct array (AFTER dropout)
%   t        : current sim time (seconds) [optional]
%
% This function deduces for each ICAO if the message at this second was kept or dropped.

    % Nothing to do if no in-range messages this step
    if isempty(preMsgs)
        return;
    end

    % Unique ICAO lists (normalize to UPPER strings)
    preICAO  = unique(upper(string({preMsgs.ICAO})));
    postICAO = unique(upper(string({postMsgs.ICAO})));

    % Build a quick membership mask: for each pre target, was it kept?
    isKept = ismember(preICAO, postICAO);

    % Global counters
    analytics.global.expected = analytics.global.expected + numel(preICAO);
    analytics.global.kept     = analytics.global.kept     + nnz(isKept);
    analytics.global.dropped  = analytics.global.dropped  + nnz(~isKept);

    % Per-target updates
    for k = 1:numel(preICAO)
        key = char(preICAO(k));
        s = localGetTargetState(analytics, key);

        s.expected = s.expected + 1;

        if isKept(k)
            s.kept = s.kept + 1;

            % Close an outage if we were in one
            if s.inOutage
                s.outageLengths(end+1,1) = s.currentOutageLen; %#ok<AGROW>
                s.inOutage = false;
                s.currentOutageLen = 0;
            end

            % Accumulate good-run length
            s.currentGoodLen = s.currentGoodLen + 1;

        else
            s.dropped = s.dropped + 1;

            % Close a good run if one was active
            if s.currentGoodLen > 0
                s.goodRunLengths(end+1,1) = s.currentGoodLen; %#ok<AGROW>
                s.currentGoodLen = 0;
            end

            % Accumulate outage
            if s.inOutage
                s.currentOutageLen = s.currentOutageLen + 1;
            else
                s.inOutage = true;
                s.currentOutageLen = 1;
            end
        end

        s.lastSeenTime = t;
        analytics.perTarget(key) = s;
    end
end

function s = localGetTargetState(analytics, key)
    if isKey(analytics.perTarget, key)
        s = analytics.perTarget(key);
        return;
    end

    % Initialize a new target state
    s = struct( ...
        'expected', 0, ...
        'kept',     0, ...
        'dropped',  0, ...
        'inOutage', false, ...
        'currentOutageLen', 0, ...
        'outageLengths', zeros(0,1), ...
        'currentGoodLen', 0, ...
        'goodRunLengths', zeros(0,1), ...
        'lastSeenTime', [] );
end