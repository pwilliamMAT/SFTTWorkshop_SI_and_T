function dropper = helperMakeAdsbDropper(varargin)
% makeAdsbDropper  Configure dropout model and state.
%
% Usage:
%   dropper = makeAdsbDropper('model','gilbert', ...
%                             'pDropG',0.01,'pDropB',0.6,'pGB',0.004,'pBG',0.33);
%   dropper = makeAdsbDropper('model','bernoulli','pDrop',0.05);
%
% Fields:
%   model   : 'gilbert' (default) or 'bernoulli'
%   pDropG  : per-message drop prob in Good state (gilbert)
%   pDropB  : per-message drop prob in Bad  state (gilbert)
%   pGB     : Good->Bad transition prob per message
%   pBG     : Bad->Good transition prob per message
%   pDrop   : per-message drop prob (bernoulli)
%
% Keep the returned struct 'dropper' and pass to stepAdsbDropper each update.

    p = inputParser;
    addParameter(p,'model','gilbert');
    addParameter(p,'pDropG',0.01);
    addParameter(p,'pDropB',0.60);
    addParameter(p,'pGB',0.004);  % tuned for 1 Hz default
    addParameter(p,'pBG',0.33);   % tuned for 1 Hz default
    addParameter(p,'pDrop',0.05);
    parse(p,varargin{:});
    cfg = p.Results;

    dropper.model = validatestring(cfg.model,{'gilbert','bernoulli'});
    dropper.cfg   = cfg;

    % Per-target state (keyed by ICAO string)
    dropper.perTarget = containers.Map('KeyType','char','ValueType','any');

    % Optional statistics
    dropper.stats.totalSeen   = 0;
    dropper.stats.totalKept   = 0;
    dropper.stats.byICAO      = containers.Map('KeyType','char','ValueType','any');
end