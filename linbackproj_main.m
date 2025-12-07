%% LINEAR BACK-PROJECTION TRIANGULATION MAIN SCRIPT
% Linear Triangulation Method (Hartley-Zisserman, Chapter 11/12)

clear all; close all; clc;

rng(42);  % Use any integer (42, 123, 2024, etc.)
% This ensures RANSAC uses the same random samples every time

%% ===== LOAD CALIBRATION DATA =====
fprintf('\n========================================\n');
fprintf('LINEAR BACK-PROJECTION ASSIGNMENT\n');
fprintf('========================================\n');

% Load images and calibration parameters from calib.m
calib;  % This loads: im1, im2, cam1, cam2, doffs, baseline, width, height

fprintf('\n Images and calibration loaded\n');
fprintf('  Image 1 size: %d x %d\n', size(im1,2), size(im1,1));
fprintf('  Image 2 size: %d x %d\n', size(im2,2), size(im2,1));
fprintf('  Intrinsic matrix K1:\n');
disp(cam1);
fprintf('  Intrinsic matrix K2:\n');
disp(cam2);

%% ===== STEP 1: FEATURE DETECTION AND MATCHING =====
fprintf('\n[STEP 1] Feature Detection and Matching...\n');

% Convert it to grayscale if needed
if size(im1, 3) == 3
    im1_gray = rgb2gray(im1);
    im2_gray = rgb2gray(im2);
else
    im1_gray = im1;
    im2_gray = im2;
end

% Detect features using SIFT (or SURF as alternative)
% Using detectSIFTFeatures for robust point detection
try
    points1 = detectSIFTFeatures(im1_gray, 'ContrastThreshold', 0.003);
    points2 = detectSIFTFeatures(im2_gray, 'ContrastThreshold', 0.003);
    
    % Extract descriptors
    [descriptors1, validPoints1] = extractFeatures(im1_gray, points1);
    [descriptors2, validPoints2] = extractFeatures(im2_gray, points2);
    
    % Match features using nearest neighbor
    indexPairs = matchFeatures(descriptors1, descriptors2, 'Unique', true, ...
        'MatchThreshold', 1.0);
    
    % Get matched points
    matchedPoints1 = validPoints1(indexPairs(:,1));
    matchedPoints2 = validPoints2(indexPairs(:,2));
    
catch
    % Fallback: Use SURF or ORB if SIFT not available
    fprintf('SIFT not available. Using SURF instead.\n');
    points1 = detectSURFFeatures(im1_gray);
    points2 = detectSURFFeatures(im2_gray);
    
    [descriptors1, validPoints1] = extractFeatures(im1_gray, points1);
    [descriptors2, validPoints2] = extractFeatures(im2_gray, points2);
    
    indexPairs = matchFeatures(descriptors1, descriptors2, 'Unique', true);
    matchedPoints1 = validPoints1(indexPairs(:,1));
    matchedPoints2 = validPoints2(indexPairs(:,2));
end

% Remove outliers using RANSAC
num_matches = length(indexPairs);
fprintf('  Initial matches: %d\n', num_matches);

% Estimate fundamental matrix using RANSAC
[F_ransac, inliers] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, ...
    'Method', 'RANSAC', 'NumTrials', 10000, 'DistanceThreshold', 0.1);

% Keep only inlier matches
matchedPoints1_inliers = matchedPoints1(inliers);
matchedPoints2_inliers = matchedPoints2(inliers);
num_inliers = length(inliers);

fprintf('  Inlier matches (RANSAC): %d\n', num_inliers);
fprintf('  Inlier percentage: %.2f%%\n\n', 100*num_inliers/num_matches);

% Extract coordinates as Nx2 matrices
m1 = [matchedPoints1_inliers.Location];  % N x 2
m2 = [matchedPoints2_inliers.Location];  % N x 2

%% ===== VISUALIZE MATCHING POINTS =====
fprintf('Visualizing matching points...\n');

fig1 = figure('Name', 'Matching Points', 'NumberTitle', 'off');
showMatchedFeatures(im1_gray, im2_gray, matchedPoints1_inliers, ...
    matchedPoints2_inliers, 'falsecolor');
title('Matched Feature Points (RANSAC filtered)');
drawnow;

%% ===== STEP 2: ESTIMATE FUNDAMENTAL MATRIX =====
fprintf('\n[STEP 2] Fundamental Matrix Estimation\n');
fprintf('  Fundamental Matrix F:\n');
disp(F_ransac);

%% ===== STEP 3: ESTIMATE CAMERA POSES =====
fprintf('\n[STEP 3] Estimating Relative Camera Poses...\n');

% Set up camera projection matrices
% K matrices from calibration
K1 = cam1;
K2 = cam2;

% Convert fundamental matrix to essential matrix
% E = K2' * F * K1
E = K2' * F_ransac * K1;

fprintf('  Essential Matrix E:\n');
disp(E);

% Decompose essential matrix to get R and t
% [U, S, V] = svd(E)
% Possible solutions: 
%   P1 = K1[I|0]
%   P2 = K2[R1|t1] where R1=U*W*V' or U*W'*V', t1=U(:,3) or -U(:,3)

[U, ~, V] = svd(E);
W = [0 -1 0; 1 0 0; 0 0 1];

% Four possible solutions
R_options = {U*W*V', U*W*V', U*W'*V', U*W'*V'};
t_options = {U(:,3), -U(:,3), U(:,3), -U(:,3)};

% Use triangulation to find valid solution
% Valid solution: most 3D points have positive depth

best_solution = 1;
max_positive_depth = 0;

for sol = 1:4
    R_test = R_options{sol};
    t_test = t_options{sol};
    
    % Ensure determinant = 1 (proper rotation)
    if det(R_test) < 0
        R_test = -R_test;
        t_test = -t_test;
    end
    
    % Set up projection matrices
    P1 = K1 * [eye(3), zeros(3,1)];
    P2 = K2 * [R_test, t_test];
    
    % Triangulate a few points to check depth
    num_test = min(20, size(m1, 1));
    points_4d = zeros(4, num_test);
    
    for i = 1:num_test
        A = [m1(i,1)*P1(3,:) - P1(1,:);
             m1(i,2)*P1(3,:) - P1(2,:);
             m2(i,1)*P2(3,:) - P2(1,:);
             m2(i,2)*P2(3,:) - P2(2,:)];
        
        [~, ~, V_tri] = svd(A);
        points_4d(:,i) = V_tri(:,end);
    end
    
    % Check depth in first camera (Z should be positive)
    depths = points_4d(3,:) ./ points_4d(4,:);
    positive_depth = sum(depths > 0);
    
    fprintf('  Solution %d: %d/%d points with positive depth\n', ...
        sol, positive_depth, num_test);
    
    if positive_depth > max_positive_depth
        max_positive_depth = positive_depth;
        best_solution = sol;
    end
end

R = R_options{best_solution};
t = t_options{best_solution};

% Ensure determinant = 1 (proper rotation)
if det(R) < 0
    R = -R;
    t = -t;
end

fprintf('\n  Selected Solution %d\n', best_solution);
fprintf('  Rotation Matrix R:\n');
disp(R);
fprintf('  Translation Vector t:\n');
disp(t');

%% ===== STEP 4 & 5 & 6: LINEAR TRIANGULATION FOR ALL POINTS =====
fprintf('\n[STEP 4-6] Linear Triangulation for All Points...\n');

% Set up projection matrices
P1 = K1 * [eye(3), zeros(3,1)];
P2 = K2 * [R, t];

num_points = size(m1, 1);
points_3d = zeros(num_points, 3);  % Store as Nx3 for convenience

% Triangulate each matching point pair
for i = 1:num_points
    % Build A matrix for this correspondence
    x1 = m1(i, 1);
    y1 = m1(i, 2);
    x2 = m2(i, 1);
    y2 = m2(i, 2);
    
    A = [x1*P1(3,:) - P1(1,:);
         y1*P1(3,:) - P1(2,:);
         x2*P2(3,:) - P2(1,:);
         y2*P2(3,:) - P2(2,:)];
    
    % Solve using SVD: A*M = 0
    [~, ~, V] = svd(A);
    M_homogeneous = V(:, end);  % Last column of V
    
    % Convert from homogeneous to 3D Euclidean coordinates
    M_3d = M_homogeneous(1:3) / M_homogeneous(4);
    
    points_3d(i, :) = M_3d';
    
    if mod(i, 100) == 0
        fprintf('  Triangulated %d/%d points\n', i, num_points);
    end
end

fprintf('   Triangulated all %d points\n\n', num_points);

% Remove points with very large depths (outliers/artifacts)
max_depth = 10000;  % Maximum reasonable depth
valid_points = points_3d(:,3) > 0 & points_3d(:,3) < max_depth;
points_3d_valid = points_3d(valid_points, :);

fprintf('  Points with valid depth (0 < Z < %d): %d/%d\n', ...
    max_depth, sum(valid_points), num_points);

%% ===== STEP 7: RENDER POINT CLOUD =====
fprintf('\n[STEP 7] Rendering Point Cloud...\n');

% Create figure for point cloud
fig2 = figure('Name', '3D Point Cloud (Linear Triangulation)', ...
    'NumberTitle', 'off', 'Position', [100 100 900 700]);

scatter3(points_3d_valid(:,1), points_3d_valid(:,2), points_3d_valid(:,3), ...
    8, points_3d_valid(:,3), 'filled');
colormap(hot);
colorbar;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('3D Point Cloud from Linear Triangulation');
grid on;
axis equal;
view(45, 30);
drawnow;

fprintf('   Point cloud rendered\n\n');

%% ===== COMPARISON: MATLAB BUILT-IN TRIANGULATE FUNCTION =====
fprintf('[STEP 8] Comparison with MATLAB triangulate() function...\n');

try
    % Try new version first (MATLAB R2021a+)
    camera1 = cameraIntrinsics(cam1(1,1), [cam1(1,3), cam1(2,3)], [size(im1,2), size(im1,1)]);
    camera2 = cameraIntrinsics(cam2(1,1), [cam2(1,3), cam2(2,3)], [size(im2,2), size(im2,1)]);
    
    points_3d_matlab = triangulate(matchedPoints1_inliers, matchedPoints2_inliers, ...
        camera1, camera2, 'Linear');
    
    fprintf('  (Using cameraIntrinsics method)\n');
    
catch
    try
        % Try older version method
        camera1 = cameraMatrix(cam1, eye(3), zeros(3,1));
        camera2 = cameraMatrix(cam2, R, t);
        
        points_3d_matlab = triangulate(matchedPoints1_inliers, matchedPoints2_inliers, ...
            camera1, camera2);
        
        fprintf('  (Using cameraMatrix method)\n');
        
    catch
        % If both fail, use basic triangulation
        fprintf('  (Warning: Using fallback triangulation method)\n');
        points_3d_matlab = linbackproj(m1, m2, P1, P2);
    end
end

% Remove outliers
valid_matlab = points_3d_matlab(:,3) > 0 & points_3d_matlab(:,3) < max_depth;
points_3d_matlab_valid = points_3d_matlab(valid_matlab, :);

fprintf('  Valid points from MATLAB triangulate: %d/%d\n', ...
    sum(valid_matlab), num_points);

% Create comparison figure
fig3 = figure('Name', '3D Point Cloud Comparison', ...
    'NumberTitle', 'off', 'Position', [1000 100 1200 700]);

subplot(1,2,1);
scatter3(points_3d_valid(:,1), points_3d_valid(:,2), points_3d_valid(:,3), ...
    8, points_3d_valid(:,3), 'filled');
colormap(hot);
colorbar;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Our Implementation (Linear Triangulation)');
grid on;
axis equal;
view(45, 30);

subplot(1,2,2);
scatter3(points_3d_matlab_valid(:,1), points_3d_matlab_valid(:,2), ...
    points_3d_matlab_valid(:,3), 8, points_3d_matlab_valid(:,3), 'filled');
colormap(hot);
colorbar;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('MATLAB Built-in triangulate()');
grid on;
axis equal;
view(45, 30);

fprintf('  Comparison figure created\n\n');

%% ===== SIGNAL-TO-NOISE RATIO (SNR) CALCULATION =====
fprintf('[STEP 9] Signal-to-Noise Ratio (SNR) Calculation...\n');

valid_in_both = valid_points & valid_matlab;

points_our = points_3d(valid_in_both, :);
points_matlab_ref = points_3d_matlab(valid_in_both, :);

n_compare = size(points_our, 1);

fprintf('  Total triangulated points: %d\n', num_points);
fprintf('  Valid in your implementation: %d\n', sum(valid_points));
fprintf('  Valid in MATLAB implementation: %d\n', sum(valid_matlab));
fprintf('  Valid in BOTH (used for SNR): %d\n', n_compare);

if n_compare < 10
    fprintf('  ERROR: Not enough valid points for comparison\n');
    SNR_linear = 0;
else
    % Direct element-by-element comparison (NO SORTING!)
    % points_our(i,:) corresponds to points_matlab_ref(i,:)
    point_errors = points_our - points_matlab_ref;
    error_distances = sqrt(sum(point_errors.^2, 2));
    
    % SNR Calculation
    % Signal Power = average squared magnitude of reference (ground truth) points
    signal_power = mean(sum(points_matlab_ref.^2, 2));
    
    % Noise Power = average squared error distance
    noise_power = mean(error_distances.^2);
    
    if noise_power > 1e-14
        SNR_linear = 10 * log10(signal_power / noise_power);
    else
        SNR_linear = 150;  % Nearly perfect match (noise is machine epsilon)
    end
    
    fprintf('\n=== SNR RESULTS ===\n');
    fprintf('  Points compared: %d\n', n_compare);
    fprintf('  Mean error distance: %.6f mm\n', mean(error_distances));
    fprintf('  Median error distance: %.6f mm\n', median(error_distances));
    fprintf('  Std error distance: %.6f mm\n', std(error_distances));
    fprintf('  Max error distance: %.6f mm\n', max(error_distances));
    fprintf('  Min error distance: %.6f mm\n', min(error_distances));
    fprintf('\n  Signal Power: %.6e\n', signal_power);
    fprintf('  Noise Power: %.6e\n', noise_power);
    fprintf('  SNR (dB): %.2f dB\n', SNR_linear);
    fprintf('  SNR (linear): %.4f\n\n', 10^(SNR_linear/10));
    
    % Quality assessment
    if SNR_linear > 80
        fprintf('  SNR > 80 dB - EXCELLENT! Implementation is correct!\n');
    elseif SNR_linear > 50
        fprintf('   SNR > 50 dB - Good\n');
    elseif SNR_linear > 30
        fprintf('   SNR > 30 dB - Acceptable\n');
    else
        fprintf('   SNR < 30 dB - Implementation has issues\n');
    end
end

fprintf('========================================\n\n');
%% Save results
save('triangulation_results.mat', 'points_3d_valid', 'points_3d_matlab_valid', ...
    'SNR_linear', 'F_ransac', 'P1', 'P2', 'matchedPoints1_inliers', ...
    'matchedPoints2_inliers', 'm1', 'm2');

fprintf('Results saved to triangulation_results.mat\n\n');
