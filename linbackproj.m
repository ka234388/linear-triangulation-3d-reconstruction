function points_3d = linbackproj(m1, m2, P1, P2)
%% LINEAR BACK-PROJECTION (TRIANGULATION)
% 
% Function: linbackproj
% 
% Purpose: Triangulates 3D points from corresponding 2D image points
%          using linear back-projection method
%
% Usage: points_3d = linbackproj(m1, m2, P1, P2)
%
% Inputs:
%   m1      - Nx2 matrix of 2D points in image 1 (x1, y1)
%   m2      - Nx2 matrix of 2D points in image 2 (x2, y2)
%   P1      - 3x4 camera projection matrix for camera 1
%   P2      - 3x4 camera projection matrix for camera 2
%
% Outputs:
%   points_3d - Nx3 matrix of 3D triangulated points (X, Y, Z)
%
% Mathematical Background fromm (Hartley-Zisserman, Chapter 11):
%
% For a point correspondence (x1, y1) <-> (x2, y2):
%
% The projection equations are:
%   λ1 * x1 = P1 * M
%   λ2 * x2 = P2 * M
%
% Where M = [X, Y, Z, 1]^T is the 3D point in homogeneous coordinates
%
% This gives us the system: A * M = 0
%   where A is a 4x4 matrix built as:
%
%   A = [x1*P1(3,:) - P1(1,:)  ]
%       [y1*P1(3,:) - P1(2,:)  ]
%       [x2*P2(3,:) - P2(1,:)  ]
%       [y2*P2(3,:) - P2(2,:)  ]
%
% Solution is to Use SVD decomposition
%   [U, S, V] = svd(A)
%   M = V(:, end)  (last column of V, corresponding to smallest singular value)
%

%% Input validation
if nargin < 4
    error('linbackproj: Insufficient number of input arguments');
end

if size(m1, 2) ~= 2 || size(m2, 2) ~= 2
    error('linbackproj: m1 and m2 must be Nx2 matrices');
end

if size(m1, 1) ~= size(m2, 1)
    error('linbackproj: m1 and m2 must have same number of rows');
end

if size(P1, 1) ~= 3 || size(P1, 2) ~= 4 || size(P2, 1) ~= 3 || size(P2, 2) ~= 4
    error('linbackproj: P1 and P2 must be 3x4 matrices');
end

%% Initialization
num_points = size(m1, 1);
points_3d = zeros(num_points, 3);

%% Main loop: Triangulate each correspondence
for i = 1:num_points
    % Extract coordinates
    x1 = m1(i, 1);
    y1 = m1(i, 2);
    x2 = m2(i, 1);
    y2 = m2(i, 2);
    
    % Build the 4x4 matrix A such that A*M = 0
    % Each image point gives 2 constraints
    
    % Row 1: x1 * P1(3,:) - P1(1,:)
    % This comes from: λ * x1 = (P1(1,:)*M) / (P1(3,:)*M)
    %              => x1 * P1(3,:)*M = P1(1,:)*M
    %              => (x1*P1(3,:) - P1(1,:)) * M = 0
    
    A = zeros(4, 4);
    A(1, :) = x1 * P1(3, :) - P1(1, :);
    A(2, :) = y1 * P1(3, :) - P1(2, :);
    A(3, :) = x2 * P2(3, :) - P2(1, :);
    A(4, :) = y2 * P2(3, :) - P2(2, :);
    
    % Solve using SVD: [U, S, V] = svd(A)
    % Solution is the null-space of A, i.e., the right singular vector
    % corresponding to the smallest singular value
    [~, ~, V] = svd(A);
    
    % The solution is the last column of V
    M_homogeneous = V(:, 4);  % Last column (4th column for 4x4 matrix)
    
    % Convert from homogeneous to 3D Euclidean coordinates
    % M_3d = M_homogeneous(1:3) / M_homogeneous(4)
    M_3d = M_homogeneous(1:3) / M_homogeneous(4);
    
    % Store result
    points_3d(i, :) = M_3d';
end

%% Output
% points_3d is now Nx3 matrix of 3D points

end  % End of function linbackproj
