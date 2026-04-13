function [msgsOut, dropper] = helperStepAdsbDropper(dropper, adsbMessages, t)
% stepAdsbDropper  Apply dropdown model to the current step's messages.
%
% Inputs:
%   dropper      : from makeAdsbDropper (persistent state across steps)
%   adsbMessages : Nx1 struct array (must include .ICAO)
%   t            : current simulation time (seconds) [not used but kept]
%
% Output:
%   msgsOut      : filtered struct array after simulated dropouts

    if isempty(adsbMessages)
        msgsOut = adsbMessages;
        return;
    end

    N = numel(adsbMessages);
    keepMask = true(N,1);

    % Normalize ICAO to string keys
    ICAOs = strings(N,1);
    for i = 1:N
        icao = adsbMessages(i).ICAO;
        if isstring(icao) || ischar(icao)
            ICAOs(i) = upper(string(icao));
        else
            ICAOs(i) = upper(string(icao)); % numeric OK; string() handles it
        end
    end

    switch dropper.model
        case 'gilbert'
            cfg = dropper.cfg;

            % Ensure per-target state exists
            for k = 1:numel(ICAOs)
                key = char(ICAOs(k));
                if ~isKey(dropper.perTarget, key)
                    s.state = 1; % 1=Good, 2=Bad
                    dropper.perTarget(key) = s;
                end
            end

            % Process each message in arrival order
            for i = 1:N
                key = char(ICAOs(i));
                s = dropper.perTarget(key);

                % Transition
                if s.state == 1
                    if rand < cfg.pGB
                        s.state = 2;
                    end
                else
                    if rand < cfg.pBG
                        s.state = 1;
                    end
                end

                % Drop decision for this message
                if s.state == 1
                    dropNow = rand < cfg.pDropG;
                else
                    dropNow = rand < cfg.pDropB;
                end

                keepMask(i) = ~dropNow;

                % Save state back
                dropper.perTarget(key) = s;

                % Stats
                dropper = localUpdateStats(dropper, key, keepMask(i));
            end

        case 'bernoulli'
            pDrop = dropper.cfg.pDrop;
            r = rand(N,1);
            keepMask = r >= pDrop;

            for i = 1:N
                key = char(ICAOs(i));
                dropper = localEnsureStats(dropper, key);
                dropper = localUpdateStats(dropper, key, keepMask(i));
            end
    end

    msgsOut = adsbMessages(keepMask);
end

function dropper = localUpdateStats(dropper, key, kept)
    dropper.stats.totalSeen = dropper.stats.totalSeen + 1;
    dropper.stats.totalKept = dropper.stats.totalKept + double(kept);
    if ~isKey(dropper.stats.byICAO, key)
        dropper.stats.byICAO(key) = struct('seen',0,'kept',0);
    end
    s = dropper.stats.byICAO(key);
    s.seen = s.seen + 1;
    s.kept = s.kept + double(kept);
    dropper.stats.byICAO(key) = s;
end

function dropper = localEnsureStats(dropper, key)
    if ~isKey(dropper.stats.byICAO, key)
        dropper.stats.byICAO(key) = struct('seen',0,'kept',0);
    end
end
