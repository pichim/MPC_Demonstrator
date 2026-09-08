function [M, D, gvec, f] = calculate_dynamics_furuta(theta, omega, param)
% Angular accelerations alpha = M⁻¹ (tau - D*omega - gvec - f)

    M = calculate_mass_matrix(theta, param);
    f = calculate_coriolis_terms(theta, omega, param);
    D = diag([param.b1, param.b2]);
    gvec = [0; param.g * param.m2 * param.l2 * sin(theta(2))];

end

function M = calculate_mass_matrix(theta, param)

    s2 = sin(theta(2));
    c2 = cos(theta(2));

    M11 = param.J1zz + ...
          param.m1 * param.l1^2 + ...
          param.m2 * param.L1^2 + ...
          (param.J2yy + param.m2 * param.l2^2) * s2^2 + ...
          param.J2xx * c2^2;

    M12 = param.m2 * param.L1 * param.l2 * c2;
    M21 = M12;
    M22 = param.m2 * param.l2^2 + param.J2zz;

    M = [M11, M12; M21, M22];
end

function f = calculate_coriolis_terms(theta, omega, param)

    s2 = sin(theta(2));
    s2c2 = sin(2 * theta(2));
    w1 = omega(1);
    w2 = omega(2);

    % These terms are nonlinear velocity products (Coriolis + centrifugal)
    f1 = -param.m2 * param.L1 * param.l2 * s2 * w2^2 + ...
          w1 * w2 * s2c2 * (param.m2 * param.l2^2 + param.J2yy - param.J2xx);

    f2 = 0.5 * w1^2 * s2c2 * (-param.m2 * param.l2^2 - param.J2yy + param.J2xx);

    f = [f1; f2];
end