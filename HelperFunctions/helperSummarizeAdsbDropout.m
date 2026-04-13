function summary = helperSummarizeAdsbDropout(analytics, varargin)
% summarizeAdsbDropout  Build global and per-target dropout summary.
%
% Name-Value:
%   'IncludeOpenOutages' (true) : include an outage that is active at end of run
%
% Output:
%   summary.global         : struct with overall metrics
%   summary.perTarget      : struct of per-ICAO metrics
%   summary.perTargetTable : table (one row per ICAO) with common metrics

    p = inputParser;
    addParameter(p,'IncludeOpenOutages',true,@islogical);
    parse(p,varargin{:});
    includeOpen = p.Results.IncludeOpenOutages;

    Ts = analytics.Ts;

    % Global counters
    G = struct();
    G.expected = analytics.global.expected;
    G.kept     = analytics.global.kept;
    G.dropped  = analytics.global.dropped;
    G.availability = safeDiv(G.kept, G.expected);

    % Per-target aggregation
    keys = analytics.perTarget.keys;
    K = numel(keys);

    per = struct();
    rows = cell(K,1);

    % For global outage stats
    allOutages = [];  % vector of outage lengths in seconds (across all targets)
    allGoods   = [];  % vector of good-run lengths in seconds (across all targets)

    for i = 1:K
        key = keys{i};
        s = analytics.perTarget(key);

        outages = s.outageLengths;
        goods   = s.goodRunLengths;

        % Optionally include any open (right-censored) run at the end
        if includeOpen
            if s.inOutage && s.currentOutageLen > 0
                outages = [outages; s.currentOutageLen]; %#ok<AGROW>
            end
            if s.currentGoodLen > 0
                goods = [goods; s.currentGoodLen]; %#ok<AGROW>
            end
        end

        % Convert to seconds
        outages_s = outages * Ts;
        goods_s   = goods   * Ts;

        % Metrics per target
        pt = struct();
        pt.ICAO            = key;
        pt.expected        = s.expected;
        pt.kept            = s.kept;
        pt.dropped         = s.dropped;
        pt.availability    = safeDiv(s.kept, s.expected);

        pt.outageCount     = numel(outages);
        pt.totalOutageTime = sum(outages_s);
        pt.avgOutage       = localMean(outages_s);
        pt.medianOutage    = localMedian(outages_s);
        pt.p95Outage       = localPercentile(outages_s, 95);
        pt.maxOutage       = localMax(outages_s);

        pt.meanGoodRun     = localMean(goods_s);
        pt.medianGoodRun   = localMedian(goods_s);
        pt.p95GoodRun      = localPercentile(goods_s, 95);
        pt.maxGoodRun      = localMax(goods_s);

        per.(matlab.lang.makeValidName(key)) = pt;

        % For table
        rows{i,1} = {pt.ICAO, pt.expected, pt.kept, pt.dropped, pt.availability, ...
                     pt.outageCount, pt.avgOutage, pt.medianOutage, pt.p95Outage, pt.maxOutage, ...
                     pt.meanGoodRun, pt.medianGoodRun, pt.p95GoodRun, pt.maxGoodRun};

        % Accumulate global vectors
        allOutages = [allOutages; outages_s]; %#ok<AGROW>
        allGoods   = [allGoods;   goods_s];   %#ok<AGROW>
    end

    % Global outage/good-run metrics
    G.outageCount       = numel(allOutages);
    G.totalOutageTime   = sum(allOutages);
    G.avgOutage         = localMean(allOutages);
    G.medianOutage      = localMedian(allOutages);
    G.p95Outage         = localPercentile(allOutages, 95);
    G.maxOutage         = localMax(allOutages);

    G.meanGoodRun       = localMean(allGoods);
    G.medianGoodRun     = localMedian(allGoods);
    G.p95GoodRun        = localPercentile(allGoods, 95);
    G.maxGoodRun        = localMax(allGoods);

    % Package outputs
    summary.global = G;
    summary.perTarget = per;

    % Build per-target table
    if K > 0
        newRows = vertcat(rows{:});  % becomes 20x14 cell
        T = cell2table(newRows, 'VariableNames', { ...
            'ICAO','Expected','Kept','Dropped','Availability', ...
            'OutageCount','AvgOutage_s','MedianOutage_s','P95Outage_s','MaxOutage_s', ...
            'MeanGoodRun_s','MedianGoodRun_s','P95GoodRun_s','MaxGoodRun_s'});
    else
        T = cell2table(cell(0,14), 'VariableNames', { ...
            'ICAO','Expected','Kept','Dropped','Availability', ...
            'OutageCount','AvgOutage_s','MedianOutage_s','P95Outage_s','MaxOutage_s', ...
            'MeanGoodRun_s','MedianGoodRun_s','P95GoodRun_s','MaxGoodRun_s'});
    end
    summary.perTargetTable = T;
end

% ---- helpers (no toolboxes required) ----
function y = safeDiv(a,b)
    if b == 0, y = NaN; else, y = a/b; end
end

function m = localMean(x)
    if isempty(x), m = NaN; else, m = mean(x); end
end

function m = localMedian(x)
    if isempty(x), m = NaN; else, m = median(x); end
end

function m = localMax(x)
    if isempty(x), m = NaN; else, m = max(x); end
end

function v = localPercentile(x, p)
    % Linear interpolation between order statistics
    if isempty(x)
        v = NaN; return;
    end
    x = sort(x(:));
    n = numel(x);
    if n == 1
        v = x; return;
    end
    % p is in [0,100]
    idx = (p/100)*(n-1) + 1;
    lo = floor(idx);
    hi = ceil(idx);
    if lo == hi
        v = x(lo);
    else
        frac = idx - lo;
        v = x(lo) + frac*(x(hi) - x(lo));
    end
end