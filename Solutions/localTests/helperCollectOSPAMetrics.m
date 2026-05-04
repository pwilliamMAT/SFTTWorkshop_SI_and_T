function [radarospa,adsbospa,fusedospa] = helperCollectOSPAMetricsAndPlot(ospa,results)

% Filter truths to moving targets only (drop first 2 platforms = sensor towers)
truthsFilteredNEDAll = cell(size(results.truePoseNEDAll));
for i = 1:numel(results.truePoseNEDAll)
    tp = results.truePoseNEDAll{i};
    if numel(tp) > 2
        tp = tp(2:end); % Changed from 3, new truePoseNEDAll has 1 fewer platform
    else
        tp = tp([]);
    end
    truthsFilteredNEDAll{i} = tp;
end

% Radar OSPA
N_ospa = numel(results.radarTrackLog);
radarospa = nan(N_ospa,1); radarLoc = nan(N_ospa,1); radarCard = nan(N_ospa,1);
for i = 1:N_ospa
    tracks = results.radarTrackLog{i};
    truths = truthsFilteredNEDAll{i};
    if isempty(tracks) && isempty(truths), continue; end
    [radarospa(i), radarLoc(i), radarCard(i)] = ospa(tracks, truths);
end

% ADS-B OSPA
N_adsb = numel(results.adsbTrackLogNED);
adsbospa = nan(N_adsb,1); adsbLoc = nan(N_adsb,1); adsbCard = nan(N_adsb,1);
reset(ospa);
for i = 1:N_adsb
    tracks = results.adsbTrackLogNED{i};
    truths = truthsFilteredNEDAll{i};
    if isempty(tracks) && isempty(truths), continue; end
    [adsbospa(i), adsbLoc(i), adsbCard(i)] = ospa(tracks, truths);
end

% Fused OSPA
N_fused = numel(results.fusedTrackLog);
fusedospa = nan(N_fused,1); fuseLoc = nan(N_fused,1); fuseCard = nan(N_fused,1);
reset(ospa);
for i = 1:N_fused
    tracks = results.fusedTrackLog{i};
    truths = truthsFilteredNEDAll{i};
    if isempty(tracks) && isempty(truths), continue; end
    [fusedospa(i), fuseLoc(i), fuseCard(i)] = ospa(tracks, truths);
end
end

radarclr = [255 255 17]/255;
adsbclr  = [183 70 255]/255;
fusedclr = [255 105 41]/255;

figure
hold on
plot((0:(N_ospa-1))/60, radarospa, "Color",radarclr);
plot((0:(N_adsb-1))/60, adsbospa, "Color",adsbclr);
plot((0:(N_fused-1))/60, fusedospa, "Color",fusedclr);
l=legend('Primary (Radar)','Secondary (ADS-B)','Fused');
l.Color = [0.1 0.1 0.1];
l.TextColor = [1 1 1];
xlabel('Time (min)')
ylabel('OSPA')
ax = gca;
grid on; box on;
ax.Color = [0.1 0.1 0.1];
ax.GridColor = [1 1 1];