function mapViewer = helperPlotTracks(sensorMode,ind,mapViewer,scenario, adsbTracksPlot,radarTracks,fusedTracks)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

% Labels and colors
adsblabel = "       ADS-B";
radarlabel = "  Radar";
fusedlabel = string(sprintf('%s\n',"","Fused"));
adsbclr = [183 70 255]/255;
radarclr = [255 255 17]/255;
fusedclr = [255 105 41]/255;

if ind == 1
    plotPlatform(mapViewer,[scenario.Platforms{3:end}],TrajectoryMode="Full");
end

switch sensorMode
    case 1  % Radar only
        if ~isempty(radarTracks)
            plotTrack(mapViewer, radarTracks, 'LabelStyle','Custom', ...
                "CustomLabel", repmat(radarlabel, 1, numel(radarTracks)), ...
                'Color', repmat(radarclr, numel(radarTracks), 1), ...
                'LineWidth', 3);
        end

    case 2  % ADS-B only (use ADS-B converted to NED for visualization)
        if ~isempty(adsbTracksPlot)
            plotTrack(mapViewer, adsbTracksPlot', 'LabelStyle','Custom', ...
                "CustomLabel", repmat(adsblabel, 1, numel(adsbTracksPlot)), ...
                'Color', repmat(adsbclr, numel(adsbTracksPlot), 1), ...
                'LineWidth', 3);
        end

    case 3  % Both with fusion
        if ~exist('adsbTracksPlot','var')
            allTracks = [radarTracks; fusedTracks];
            allLabels = [repmat(radarlabel, 1, numel(radarTracks)), ...
                repmat(fusedlabel, 1, numel(fusedTracks))];
            allColors = [repmat(radarclr, numel(radarTracks), 1); ...
                repmat(fusedclr, numel(fusedTracks), 1)];
        else
            allTracks = [adsbTracksPlot'; radarTracks; fusedTracks];
            allLabels = [repmat(adsblabel, 1, numel(adsbTracksPlot)), ...
                repmat(radarlabel, 1, numel(radarTracks)), ...
                repmat(fusedlabel, 1, numel(fusedTracks))];
            allColors = [repmat(adsbclr, numel(adsbTracksPlot), 1); ...
                repmat(radarclr, numel(radarTracks), 1); ...
                repmat(fusedclr, numel(fusedTracks), 1)];
        end

        if ~isempty(allTracks)
            plotTrack(mapViewer, allTracks, 'LabelStyle','Custom', ...
                "CustomLabel", allLabels, ...
                'Color', allColors, ...
                'LineWidth', 3);
        end
end
end