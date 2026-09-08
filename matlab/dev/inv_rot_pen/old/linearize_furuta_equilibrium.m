function [A,B,C,D] = linearize_furuta_equilibrium(theta_eq, param, actuated)
% x = [theta1; theta2; omega1; omega2]
% u = [tau1] (default) or [tau1; tau2] if actuated='both'
% Returns continuous-time (A,B,C,D).

    if nargin < 3, actuated = 'tau1'; end

    omega_eq = [0;0];

    % Evaluate M and D at the equilibrium angle
    [M, Dmat, ~, ~] = calculate_dynamics_furuta(theta_eq, omega_eq, param);

    % Gravity Jacobian ∂g/∂theta at theta_eq (analytical)
    dgdtheta = [0,                                           0;
                0,  param.g * param.m2 * param.l2 * cos(theta_eq(2))];

    % Assemble A
    A = zeros(4);
    A(1:2,3:4) = eye(2);              % dtheta/dt = omega
    A(3:4,1:2) = - (M \ dgdtheta);    % stiffness-like gravity term
    A(3:4,3:4) = - (M \ Dmat);        % damping term

    % Assemble B
    switch actuated
        case 'tau1'
            Btau = [1;0];
        case 'both'
            Btau = eye(2);
        otherwise
            error('actuated must be ''tau1'' or ''both''.');
    end
    B = zeros(4, size(Btau,2));
    B(3:4,:) = M \ Btau;

    % Default output matrices (full state, direct feedthrough zero)
    C = eye(4);
    D = zeros(size(C,1), size(B,2));
end


% function [A, B] = linearize_furuta_equilibrium(theta_eq, param)
%     % Linearize Furuta pendulum around equilibrium (theta_eq, omega = 0)
% 
%     omega_eq = [0; 0];
%     [M, D, gvec, ~] = calculate_dynamics_furuta(theta_eq, omega_eq, param);
% 
%     % State matrix A: 4x4
%     A = zeros(4);
%     A(1:2, 3:4) = eye(2);   % dtheta/dt = omega
% 
%     % % Derivative of gravity vector with respect to theta
%     % dg_dtheta = zeros(2);
%     % eps = 1e-6;
%     % for i = 1:2
%     %     dtheta = zeros(2,1);
%     %     dtheta(i) = eps;
%     %     g_plus = [0; param.g * param.m2 * param.l2 * sin(theta_eq(2) + dtheta(2))];
%     %     g_minus = [0; param.g * param.m2 * param.l2 * sin(theta_eq(2) - dtheta(2))];
%     %     dg_dtheta(:,i) = (g_plus - g_minus) / (2 * eps);
%     % end
% 
%     % Derivative of gravity vector with respect to theta (analytical)
%     dg_dtheta = [[0,                                                0] ; ...
%                  [0, param.g * param.m2 * param.l2 * cos(theta_eq(2))]];
% 
%     A(3:4, 1:2) = -M \ dg_dtheta;   % theta dynamics from gravity
%     A(3:4, 3:4) = -M \ D;           % damping contribution
% 
%     % Input matrix B: only tau1 is actuated
%     B = zeros(4,1);
%     B(3:4) = M \ [1; 0];
% end
