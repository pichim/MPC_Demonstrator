#pragma once
#include "matrices_prep.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace mpc_prep_curr {

struct Params {
    double Ts = 1e-3;
    int Np = 20;
    int Nc = 5;

    double k_m = 97.5e-3;
    double i0 = 41.6e-3;
    double w0 = 2.0 * M_PI * 5840.0 / 60.0;
    double J_M = 71.7e-7;
    double J_L = 0.0;

    double Q = 100.0;
    double Rw = 10.0;
    Eigen::Vector3d Qndiag = Eigen::Vector3d(0.0211, 5.186e-05, 8.441e-06);
    double Rn = 1e-4;

    double wmx = 100.0;
    double imx = 1.5;
    double Wslack = 1e6;
};

inline mpc_prep::Matrices compute(const Params &p)
{
    if (p.Nc < 1 || p.Np < p.Nc || p.Ts <= 0 || p.k_m == 0 || p.w0 <= 0 || p.i0 < 0 ||
        p.J_M + p.J_L <= 0 || p.Q <= 0 || p.Rw <= 0 || p.Rn <= 0 || p.Wslack <= 0 ||
        p.wmx <= 0 || p.imx <= 0)
        throw std::invalid_argument("matrices_prep_curr: invalid parameter");

    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const int Np = p.Np;
    const int Nc = p.Nc;
    const int n = 2;
    const double J = p.J_M + p.J_L;
    const double b = p.k_m * p.i0 / p.w0;

    MatrixXd Mc = MatrixXd::Zero(3, 3);
    Mc << -b / J, 0, p.k_m / J,
           1,     0, 0,
           0,     0, 0;
    const MatrixXd M0 = mpc_prep::expm(Mc * p.Ts);

    const MatrixXd A = M0.topLeftCorner(n, n);
    const VectorXd B = M0.block(0, n, n, 1);
    const VectorXd Bd = B;

    mpc_prep::Matrices m;
    m.umx = p.imx;

    m.Aaug = MatrixXd::Zero(n + 1, n + 1);
    m.Aaug.topLeftCorner(n, n) = A;
    m.Aaug.block(0, n, n, 1) = Bd;
    m.Aaug(n, n) = 1.0;
    m.Baug = VectorXd::Zero(n + 1);
    m.Baug.head(n) = B;
    m.Caug = MatrixXd::Zero(1, n + 1);
    m.Caug(0, 1) = 1.0;

    const MatrixXd Qn = MatrixXd(p.Qndiag.asDiagonal());
    MatrixXd Rn(1, 1);
    Rn(0, 0) = p.Rn;
    const MatrixXd P = mpc_prep::filterRiccati(m.Aaug, m.Caug, Qn, Rn);
    const MatrixXd S = m.Caug * P * m.Caug.transpose() + Rn;
    m.Lgain = m.Aaug * P * m.Caug.transpose() * S.inverse();

    std::vector<MatrixXd> pw(Np + 1);
    pw[0] = MatrixXd::Identity(n, n);
    for (int k = 1; k <= Np; ++k) pw[k] = A * pw[k - 1];

    MatrixXd Sx(n * Np, n), Su = MatrixXd::Zero(n * Np, Np);
    VectorXd Sd(n * Np);
    MatrixXd Asum = MatrixXd::Zero(n, n);
    for (int i = 1; i <= Np; ++i) {
        const int r = (i - 1) * n;
        Sx.block(r, 0, n, n) = pw[i];
        Asum += pw[i - 1];
        Sd.segment(r, n) = Asum * Bd;
        for (int j = 1; j <= i; ++j) Su.block(r, j - 1, n, 1) = pw[i - j] * B;
    }

    MatrixXd Cpos = MatrixXd::Zero(Np, n * Np);
    MatrixXd Cvel = MatrixXd::Zero(Np, n * Np);
    for (int k = 0; k < Np; ++k) {
        Cvel(k, n * k) = 1.0;
        Cpos(k, n * k + 1) = 1.0;
    }

    const MatrixXd Yx = Cpos * Sx;
    const MatrixXd Yu_full = Cpos * Su;
    const VectorXd Yd = Cpos * Sd;
    const MatrixXd Vx = Cvel * Sx;
    const MatrixXd Vu_full = Cvel * Su;
    const VectorXd Vd = Cvel * Sd;

    MatrixXd Lc = MatrixXd::Zero(Nc, Nc);
    for (int i = 0; i < Nc; ++i)
        for (int j = 0; j <= i; ++j) Lc(i, j) = 1.0;

    MatrixXd Tu = MatrixXd::Zero(Np, Nc);
    Tu.topLeftCorner(Nc, Nc).setIdentity();
    for (int i = Nc; i < Np; ++i) Tu(i, Nc - 1) = 1.0;

    const MatrixXd Tdu = Tu * Lc;
    const VectorXd Tu0 = Tu * VectorXd::Ones(Nc);

    const MatrixXd Yu = Yu_full * Tdu;
    const VectorXd Yuu = Yu_full * Tu0;
    const MatrixXd Vu = Vu_full * Tdu;
    const VectorXd Vuu = Vu_full * Tu0;

    const MatrixXd Huu = 2.0 * (p.Q * Yu.transpose() * Yu + p.Rw * MatrixXd::Identity(Nc, Nc));
    m.H = MatrixXd::Zero(2 * Nc, 2 * Nc);
    m.H.topLeftCorner(Nc, Nc) = Huu;
    m.H.bottomRightCorner(Nc, Nc) = 2.0 * p.Wslack * MatrixXd::Identity(Nc, Nc);

    m.Fx = 2.0 * p.Q * Yu.transpose() * Yx;
    m.Fu0 = 2.0 * p.Q * Yu.transpose() * Yuu;
    m.Fd = 2.0 * p.Q * Yu.transpose() * Yd;
    m.Fr = -2.0 * p.Q * Yu.transpose();

    const int MM = 2 * Np + 3 * Nc;
    const MatrixXd INc = MatrixXd::Identity(Nc, Nc);
    m.Aineq = MatrixXd::Zero(MM, 2 * Nc);
    m.Ex = MatrixXd::Zero(MM, n);
    m.Eu0 = VectorXd::Zero(MM);
    m.Ed = VectorXd::Zero(MM);
    m.bconst = VectorXd::Zero(MM);

    int r = 0;
    m.Aineq.block(r, 0, Np, Nc) = Vu;
    m.Ex.block(r, 0, Np, n) = -Vx;
    m.Eu0.segment(r, Np) = -Vuu;
    m.Ed.segment(r, Np) = -Vd;
    m.bconst.segment(r, Np).setConstant(p.wmx);
    r += Np;
    m.Aineq.block(r, 0, Np, Nc) = -Vu;
    m.Ex.block(r, 0, Np, n) = Vx;
    m.Eu0.segment(r, Np) = Vuu;
    m.Ed.segment(r, Np) = Vd;
    m.bconst.segment(r, Np).setConstant(p.wmx);
    r += Np;
    m.Aineq.block(r, 0, Nc, Nc) = Lc;
    m.Aineq.block(r, Nc, Nc, Nc) = -INc;
    m.Eu0.segment(r, Nc).setConstant(-1.0);
    m.bconst.segment(r, Nc).setConstant(p.imx);
    r += Nc;
    m.Aineq.block(r, 0, Nc, Nc) = -Lc;
    m.Aineq.block(r, Nc, Nc, Nc) = -INc;
    m.Eu0.segment(r, Nc).setConstant(1.0);
    m.bconst.segment(r, Nc).setConstant(p.imx);
    r += Nc;
    m.Aineq.block(r, Nc, Nc, Nc) = -INc;

    if (!m.H.allFinite() || !m.Lgain.allFinite() || !m.Aineq.allFinite())
        throw std::runtime_error("matrices_prep_curr: non-finite result");

    return m;
}

}


