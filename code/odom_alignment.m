scanStruct = load('C:\Users\Önder Kul\Desktop\Engineer\Control\AMCL_optim\laser_scan.mat');
scan_times = scanStruct.scan_times; scanArray = scanStruct.scan_array;

posesStruct = load('C:\Users\Önder Kul\Desktop\Engineer\Control\AMCL_optim\poses_data.mat');
poses = posesStruct.poses;

%% Interpolate odometry measurements to align with scan readings
xq = scan_times;
time_stamps = poses(:, 4);
x_poses = poses(:, 1);
y_poses = poses(:, 2);
theta_poses = poses(:, 3);

x_interp = interp1(time_stamps, x_poses, scan_times);
y_interp = interp1(time_stamps, y_poses, scan_times);
theta_interp = interp1(time_stamps, theta_poses, scan_times);

figure
plot(time_stamps, theta_poses, 'o', scan_times, theta_interp, ':.');
xlim auto;
title("Interpolated Angle Pose Values")

%% Find Corresponding indexes to specific timestamps beforehand
% Define the specific timestamps for which we want to find corresponding indexes
specificTimestamps =  [1776670174 1776670218 1776670257 1776670291 1776670330 1776670374];
time_idx = zeros(1, 6);
idx = 1;
for ts = specificTimestamps
    closest_idx = 1;
    min_diff = 100000; %A large number
    for i=2:1:length(x_interp)
        curr_time = scan_times(i);
        diff = abs(curr_time - ts);
        if diff < min_diff
            min_diff = diff;
            closest_idx = i;
        end
    end
    idx = idx + 1;
    time_idx(1, idx) = closest_idx;

end

time_idx(1) = [];
%% Save the ground truth values for future use
ground_truths = [[1.18 2.52 1.57079632]; [-0.75 2.57 1.57079632]; [0 0 1.57079632]; [1.18 2.52 1.57079632]; [-0.75 2.57 1.57079632]; [0 0 1.57079632]];

%% Save the Interpolated odom values for direct use in AMCL
save('C:\Users\Önder Kul\Desktop\Engineer\Control\AMCL_optim\interpolated_odom.mat', 'x_interp', 'y_interp', 'theta_interp', 'scan_times', 'time_idx', 'ground_truths');
