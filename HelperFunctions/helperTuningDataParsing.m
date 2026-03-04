%% largeTuningData is a cell array where each cell is a timetable 
% with a 'Time' column of type duration.
% This data is a full dataset for ADSB capture, for visualization in
% workshop, we used a subset of these trajectories

% 1) Compute start and end times (handles cases where Time doesn't start at 0)
tStart = cellfun(@(tt) tt.Time(1),  largeTuningData);
tEnd   = cellfun(@(tt) tt.Time(end), largeTuningData);

% 2) Duration of each dataset
dur = tEnd - tStart;   % duration array

% 3) Build mask for 99s <= duration <= 500s
mask_99_500 = dur >= seconds(99) & dur <= seconds(500);

% 4) Use the mask to index the original cell array
largeTuningData_99_500 = largeTuningData(mask_99_500);

%% Above results in 85 trajectories, index to find interesting group of trajectories for simulation
largeTuningData_99_500_first20 = largeTuningData_99_500(1:20);
largeTuningData_99_500_second20 = largeTuningData_99_500(21:40);
largeTuningData_99_500_third20 = largeTuningData_99_500(41:60);
largeTuningData_99_500_fourth20 = largeTuningData_99_500(61:80);
largeTuningData_99_500_fifth20 = largeTuningData_99_500(66:85); % This subset is used in Workshop