%% Load Scan & Pose Data from ROSBAGs
scan_struct = load('C:\Users\Önder Kul\Desktop\Engineer\Control\AMCL_optim\laser_scan.mat');
scan_times = scan_struct.scan_times; 
scan_array = scan_struct.scan_array;

odom_poses = load('interpolated_odom.mat');
x_poses = odom_poses.x_interp;
y_poses = odom_poses.y_interp;
theta_poses = odom_poses.theta_interp;
time_idx = odom_poses.time_idx;
ground_truths = odom_poses.ground_truths;

% Monte Carlo Base Implementation
%% Load the Map PGM file 
pgmFile = 'C:\Users\Önder Kul\Desktop\Engineer\Control\AMCL_optim\lab_mapping\maps\lab_parkur_map.pgm';
map = imread(pgmFile);
%%
% map = double(map) / 255;
% mapOcc = 1 - map;
% resolution = 20.0;
% origin = [-10.0, -10.0];
% map = binaryOccupancyMap(map, resolution);
% map.GridOriginInLocal= origin;
%% ROS2 Bags Initialization
ros2BagFolder = "C:\Users\Önder Kul\Desktop\Engineer\Control\AMCL_optim\lab_mapping\rosbags\rosbag2_2026_04_20-07_28_15";
bagReader = ros2bagreader(ros2BagFolder);
msgs = readMessages(bagReader);
scanSel = select(bagReader, "Topic", "/scan");
scanMsgs = readMessages(scanSel);
mapSel = select(bagReader, "Topic", "/map");
mapMsgs = readMessages(mapSel);
mapData = mapMsgs{1};
%%Loading Map
% map = rosReadOccupancyGrid(mapData);
% figure
% show
%%
map = rosReadBinaryOccupancyGrid(mapData, 50);
% figure
% show(map);
%% AMCL Initialization
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.1 0.1 0.1 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorPose = [0.3 0 0];
rangeFinderModel.RandomMeasurementWeight = 0.3;
rangeFinderModel.ExpectedMeasurementWeight = 0.7;
rangeFinderModel.MeasurementNoise = 0.2;
rangeFinderModel.SensorLimits = [0.06 8.0];
rangeFinderModel.Map = map;

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.1 0.1 0.2];
amcl.ResamplingInterval = 1;
amcl.ParticleLimits = [500 10000];

num_updates = 0;

amcl.GlobalLocalization = true;
pose_estimates = zeros(6, 3);
estimate_idx = 1;
%% AMCL Pose Update Iterations
for i = 1:1:length(scan_times)
    scanMsg = scanMsgs{i};
    scan = rosReadLidarScan(scanMsg);
    currOdom = [x_poses(i) y_poses(i) theta_poses(i)];
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(currOdom, scan);

    if estimate_idx ~= 7 && i == time_idx(estimate_idx)
        pose_estimates(estimate_idx, :) = estimatedPose(:);
        estimate_idx = estimate_idx + 1;
    end

    if isUpdated
        num_updates = num_updates + 1;
    end
end

%% Calculate RMSE by comparing estimates with ground truths
diffs = pose_estimates - ground_truths;
rmse = sqrt(mean(diffs.^2, 1));


%% Random Search Optimization of RMSE For AMCL
bounds = struct();
bounds.odomNoise = [0.03 0.25];
bounds.randWeight = [0.1 0.5];
bounds.updateDist = [0.05 0.3];
bounds.updateRot = [0.1 0.3];
bounds.MeasurementNoise = [0.1 0.4];
bounds.maxParticles = [5000 10000];

num_trials = 100; % Number of random configurations to test
results = table(); % To store performance data

params = table();
params.odomNoise = bounds.odomNoise(1) + (bounds.odomNoise(2) - bounds.odomNoise(1)) * rand(num_trials, 4);
params.randWeight = bounds.randWeight(1) + (bounds.randWeight(2) - bounds.randWeight(1)) * rand(num_trials, 1);
params.expWeight = 1 - params.randWeight;
params.updateDist = bounds.updateDist(1) + (bounds.updateDist(2) - bounds.updateDist(1)) * rand(num_trials, 2);
params.updateRot = bounds.updateRot(1) + (bounds.updateRot(2) - bounds.updateRot(1)) * rand(num_trials, 1);
params.MeasurementNoise = bounds.MeasurementNoise(1) + (bounds.MeasurementNoise(2) - bounds.MeasurementNoise(1)) * rand(num_trials, 1);
% params.maxParticles = bounds.maxParticles(1) + (bounds.maxParticles(2) - bounds.maxParticles(1)) / 20 * randi([0, 20], num_trials);

best_rmse_sum = inf;
best_idx = 0;
rmse_history = zeros(num_trials, 1);

%% Run the Optimization Loop

for t=1:num_trials
    %Initialize Search Params
    release(amcl);
    odometryModel.Noise = params.odomNoise(t, :);
    rangeFinderModel.RandomMeasurementWeight = params.randWeight(t, :);
    rangeFinderModel.ExpectedMeasurementWeight = 1- params.randWeight(t);
    amcl.UpdateThresholds = [params.updateDist(t, :) params.updateRot(t)];
    rangeFinderModel.MeasurementNoise = params.MeasurementNoise(t);
    amcl.ParticleLimits = [500 10000];

    %Initialize search
    pose_estimates = zeros(6, 3);
    estimate_idx = 1;

    for i = 1:1:length(scan_times)
        scanMsg = scanMsgs{i};
        scan = rosReadLidarScan(scanMsg);
        currOdom = [x_poses(i) y_poses(i) theta_poses(i)];
        [isUpdated,estimatedPose, estimatedCovariance] = amcl(currOdom, scan);
    
        if estimate_idx ~= 7 && i == time_idx(estimate_idx)
            pose_estimates(estimate_idx, :) = estimatedPose(:);
            estimate_idx = estimate_idx + 1;
        end
    end

    % Objective Function: Minimize the sum of RMSE for x, y, and theta
    diffs = pose_estimates - ground_truths;
    current_rmse = sqrt(mean(diffs.^2, 1));
    total_score = sum(current_rmse);
    rmse_history(t) = total_score;
    
    if total_score < best_rmse_sum
        best_rmse_sum = total_score;
        best_idx = t;
    end
end

fprintf('\n--- Optimization Finished ---\n');
fprintf('Best RMSE Sum: %.4f\n', best_rmse_sum);
disp('Best Parameter Configuration:');
disp(params(best_idx, :));

figure;
scatter(1:num_trials, rmse_history, 'filled');
yline(best_rmse_sum, 'r--', 'Best Result');
xlabel('Trial Number'); ylabel('Sum of RMSE');
title('Random Search Convergence');

%% Try the model with the best parameters
%Initialize Search Params
release(amcl);
odometryModel.Noise = params.odomNoise(best_idx, :);
rangeFinderModel.RandomMeasurementWeight = params.randWeight(best_idx, :);
rangeFinderModel.ExpectedMeasurementWeight = 1- params.randWeight(best_idx);
amcl.UpdateThresholds = [params.updateDist(best_idx, :) params.updateRot(best_idx)];
rangeFinderModel.MeasurementNoise = params.MeasurementNoise(best_idx);
amcl.ParticleLimits = [500 10000];

%Initialize search
pose_estimates = zeros(6, 3);
estimate_idx = 1;
for i = 1:1:length(scan_times)
    scanMsg = scanMsgs{i};
    scan = rosReadLidarScan(scanMsg);
    currOdom = [x_poses(i) y_poses(i) theta_poses(i)];
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(currOdom, scan);

    if estimate_idx ~= 7 && i == time_idx(estimate_idx)
        pose_estimates(estimate_idx, :) = estimatedPose(:);
        estimate_idx = estimate_idx + 1;
    end
end

% Objective Function: Minimize the sum of RMSE for x, y, and theta
diffs = pose_estimates - ground_truths;
current_rmse = sqrt(mean(diffs.^2, 1));
%%
best_config = [params.odomNoise(best_idx, :) params.expWeight(best_idx) params.updateDist(best_idx, :) params.updateRot(best_idx) params.MeasurementNoise(best_idx)];

%% Save the best params for lma optimization starting point
save('C:\Users\Önder Kul\Desktop\Engineer\Control\AMCL_optim\random_search_best_params.mat', 'params', 'rmse_history', 'best_config')











