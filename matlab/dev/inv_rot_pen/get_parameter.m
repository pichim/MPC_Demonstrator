function param = get_parameter()

% Motor
param.R  = 4.12 * 1.1; % Motor resistor in Ohm (10% scaling ???)
param.L  = 1.31e-3;    % Motor inductance in H
param.km = 97.5e-3;    % Motor current torque constant in Nm/A

% The paper builds the structure of the inertia tensors like this:
%
% [0  0  0]
% [0 J1  0]
% [0  0 J1]
%
% [0  0  0]
% [0 J2  0]
% [0  0 J2]
%
% Most Furuta pendulums tend to have long slender arms, such
% that the moment of inertia along the axis of the arms is
% negligible. In addition, most arms have rotational symmetry,
% such that the moments of inertia in two of the principal axes
% are equal.

% Only for Visualization in Simscape
param.L0a = 0.193;
param.L0b = 0.019;
param.r0a = 0.130/2;
param.r0b = 9e-3/2;

param.g = 9.80665;

% Rotation Axis 1: Motor and everything that is attached
m1  = 122.2e-3;
J1m = 11270.84e-9; % from CAD
l1 = 0.027248;
L1 = 0.09 - 9e-3/2;

% Only for Visualization in Simscape
param.r1 = 9e-3/2;

param.m1 = m1;
param.L1 = L1;
param.l1 = l1; 
% param.J1xx = 0; % does not appear in differential equation
% param.J1yy = 0; % does not appear in differential equation
param.J1zz = J1m; % calculated so that J1m + m1*l1^2 is equal to measured inertia
param.b1 = 1e-6; % TODO: Measure

% Rotation Axis 2: Pendulum
m2 = 28e-3;
r2 = 9e-3/2;
L2 = 161e-3;
l2 = L2 / 2;

% Only for Visualization in Simscape
param.r2 = r2;

param.m2 = m2;
param.L2 = L2;
param.l2 = l2;
param.J2xx = 1/2 * m2 * r2^2; % TODO: Check if can be neglected
param.J2yy = (1/4 * m2 * r2^2 + 1/12 * m2 * L2^2);
param.J2zz = (1/4 * m2 * r2^2 + 1/12 * m2 * L2^2);
param.b2 = 4.4e-05; % from measurement

end