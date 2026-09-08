function [M, C, G, D] = calculate_dynamics_furuta(theta, omega, param)
% M(q) * qdd + C(q,qdot) * qdot + G(q) + D * qdot = tau

    % Pre-compute terms
    s2   = sin(theta(2));
    c2   = cos(theta(2));
    s2c2 = sin(2*theta(2));
    w1   = omega(1);
    w2   = omega(2);

    % M(q)
    M11 = param.J1zz + param.m1*param.l1^2 + param.m2*param.L1^2 ...
        + (param.J2yy + param.m2*param.l2^2) * s2^2 ...
        +  param.J2xx * c2^2;

    M12 =  param.m2 * param.L1 * param.l2 * c2;
    M21 =  M12;
    M22 =  param.J2zz + param.m2 * param.l2^2;

    M = [M11, M12;
         M21, M22];

    % C(q,qdot): choose C so that C*omega reproduces the velocity vector f
    % f1 = -m2 L1 l2 s2 * w2^2 + w1*w2 * s2c2 * (m2 l2^2 + J2yy - J2xx)
    % f2 =  0.5*w1^2 * s2c2 * (-m2 l2^2 - J2yy + J2xx)
    C11 =  w2 * s2c2 * (param.m2*param.l2^2 + param.J2yy - param.J2xx);
    C12 = -param.m2 * param.L1 * param.l2 * s2 * w2;   % <-- fixed (no factor 2)
    C21 =  0.5 * w1 * s2c2 * (-param.m2*param.l2^2 - param.J2yy + param.J2xx);
    C22 =  0;

    C = [C11, C12;
         C21, C22];

    % G(q)
    G = [0;
         param.m2 * param.g * param.l2 * sin(theta(2))];

    % D (viscous)
    D = diag([param.b1, param.b2]);
end
