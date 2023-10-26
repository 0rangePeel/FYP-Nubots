function plotFoot(Ab,rB,colour)
%PLOTFOOT Summary of this function goes here
%   Detailed explanation goes here

% Define rectangle parameters
width = 0.1;   % Width of the rectangle
length = 0.2;  % Height of the rectangle

A = Ab(1:3, 1:3);

theta   =         atan2( A(3,2), A(3,3));                   
phi     =         atan2(-A(3,1), sqrt(A(3,2)^2 + A(3,3)^2));  
psi     =         atan2( A(2,1), A(1,1));                  

% Rotation matrix around z-axis (phi)
Rz = [cos(phi) -sin(phi) 0;
      sin(phi) cos(phi)  0;
      0        0         1];

% Rotation matrix around y-axis (theta)
Ry = [cos(theta)  0  sin(theta);
      0           1  0;
      -sin(theta) 0  cos(theta)];

% Rotation matrix around x-axis (psi)
Rx = [1  0           0;
      0  cos(psi)  -sin(psi);
      0  sin(psi)   cos(psi)];

% Combine rotation matrices to get the overall rotation
R = Rz * Ry * Rx;

% Define rectangle vertices in the local frame (before rotation)
local_vertices = [width/2 width/2 -width/2 -width/2;
                  2*length/3 -1*length/3 -1*length/3 2*length/3;
                  0 0 0 0];

% Apply rotation to the local vertices
rotated_vertices = R * local_vertices;

% Translate the rotated vertices to the given point
translated_vertices = rotated_vertices + repmat([rB(3); rB(1); rB(2)], 1, 4);
% translated_vertices = rotated_vertices + repmat([rB4(3); rB4(1); rB4(2)], 1, 4);

% Plot the rectangle
fill3(translated_vertices(1,:), translated_vertices(2,:), translated_vertices(3,:), colour, 'FaceAlpha', 0.5);
end

