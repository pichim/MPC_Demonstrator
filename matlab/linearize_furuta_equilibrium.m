function [A,B,C,D] = linearize_furuta_equilibrium(theta_eq, param, actuated)
% Linearize the full nonlinear Furuta dynamics
% M(q) * qdd + C(q,qdot) * qdot + G(q) + D * qdot = tau
% about an equilibrium:  q = [theta1; theta2] , qdot = 0 .
%
% State vector: x = [theta1; theta2; omega1; omega2]
% Input:        u = [tau1] (default)  or  [tau1; tau2]  if actuated = 'both'

    if nargin < 3
        actuated = 'tau1';
    end

    omega_eq = [0;0]; % at rest
    [M, ~, G, Dmat] = calculate_dynamics_furuta(theta_eq,omega_eq,param);

    % Gravity Jacobian dG/dq evaluated at equilibrium
    % Only dG2/dtheta2 is non-zero:  d(m2*g*l2*sin(theta2))/dtheta2 = m2*g*l2*cos(theta2)
    dGdtheta = zeros(2);
    dGdtheta(2,2) = param.m2 * param.g * param.l2 * cos(theta_eq(2));

    % Assemble A matrix
    A = zeros(4);
    A(1:2,3:4) = eye(2);                  % dtheta/dt = omega
    A(3:4,1:2) = - M \ dGdtheta;          % stiffness-like gravity term
    A(3:4,3:4) = - M \ Dmat;              % viscous damping term

    % Assemble B matrix
    switch actuated
        case 'tau1'
            Btau = [1;0];                 % only base joint actuated
        case 'both'
            Btau = eye(2);                % both joints actuated
        otherwise
            error('actuated must be ''tau1'' or ''both''.');
    end
    B = zeros(4,size(Btau,2));
    B(3:4,:) = M \ Btau;

    % Output matrices (full-state output by default)
    C = eye(4);
    D = zeros(size(C,1),size(B,2));
end
