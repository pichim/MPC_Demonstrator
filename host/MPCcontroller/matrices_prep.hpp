#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace mpc_prep {

struct Params {
    double Ts = 1e-3;
    int Np = 20;
    int Nc = 5;

    double L = 1.31e-3;
    double R = 4.532;
    double k_m = 97.5e-3;
    double b = 6.632e-6;
    double J_M = 71.7e-7;
    double J_L = 0.0;

    Eigen::Vector3d Qdiag = Eigen::Vector3d(0.0002936, 0.0002109, 100.0);
    double Rw = 0.6548;
    Eigen::Vector4d Qndiag = Eigen::Vector4d(0.08917, 5.594e-07, 1.177e-05, 2.763e-08);
    Eigen::Vector2d Rndiag = Eigen::Vector2d(3.265e-06, 3.475e-08);

    double Wslack = 1e4;
    double imx = 1.0;
    double umx = 10.0;
};

struct Matrices {
    Eigen::MatrixXd H, Aineq, Fx, Fr, Ex, Aaug, Caug, Lgain;
    Eigen::VectorXd Fu0, Fd, Eu0, Ed, bconst, Baug;
    double umx = 0.0;
};

inline Eigen::MatrixXd expm(const Eigen::MatrixXd &M)
{
    const int n = static_cast<int>(M.rows());
    const double nrm = M.cwiseAbs().rowwise().sum().maxCoeff();
    int s = 0;
    if (nrm > 0.5) s = std::max(0, static_cast<int>(std::ceil(std::log2(nrm / 0.5))));
    const Eigen::MatrixXd Ms = M / std::pow(2.0, s);
    Eigen::MatrixXd term = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd E = term;
    for (int k = 1; k <= 20; ++k) {
        term = term * Ms / k;
        E += term;
    }
    for (int i = 0; i < s; ++i) E = E * E;
    return E;
}

inline Eigen::MatrixXd filterRiccati(const Eigen::MatrixXd &A, const Eigen::MatrixXd &C,
                                     const Eigen::MatrixXd &Qw, const Eigen::MatrixXd &Rv)
{
    const int n = static_cast<int>(A.rows());
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd Ak = A.transpose();
    Eigen::MatrixXd Gk = C.transpose() * Rv.inverse() * C;
    Eigen::MatrixXd Hk = Qw;
    bool converged = false;
    for (int it = 0; it < 200; ++it) {
        Eigen::PartialPivLU<Eigen::MatrixXd> lu(I + Gk * Hk);
        const Eigen::MatrixXd WiA = lu.solve(Ak);
        const Eigen::MatrixXd WiG = lu.solve(Gk);
        Eigen::MatrixXd Hn = Hk + Ak.transpose() * Hk * WiA;
        Eigen::MatrixXd Gn = Gk + Ak * WiG * Ak.transpose();
        Eigen::MatrixXd An = Ak * WiA;
        Hn = 0.5 * (Hn + Hn.transpose());
        Gn = 0.5 * (Gn + Gn.transpose());
        const double diff = (Hn - Hk).cwiseAbs().maxCoeff();
        const double scale = std::max(1.0, Hn.cwiseAbs().maxCoeff());
        Hk = Hn;
        Gk = Gn;
        Ak = An;
        if (!Hk.allFinite()) break;
        if (diff <= 1e-13 * scale) { converged = true; break; }
    }
    if (!converged) throw std::runtime_error("Riccati equation did not converge");
    return Hk;
}

inline Matrices compute(const Params &p)
{
    if (p.Nc < 1 || p.Np < p.Nc || p.Ts <= 0 || p.L <= 0 || p.R <= 0 || p.k_m == 0 ||
        p.J_M + p.J_L <= 0 || p.Rw <= 0 || p.Wslack <= 0 || p.imx <= 0 || p.umx <= 0 || p.b < 0)
        throw std::invalid_argument("matrices_preparation: invalid parameter");

    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const int Np = p.Np;
    const int Nc = p.Nc;
    const int nx = 3;
    const double J = p.J_M + p.J_L;

    MatrixXd Ac(3, 3), Mc = MatrixXd::Zero(5, 5);
    Ac << -p.R / p.L, -p.k_m / p.L, 0,
           p.k_m / J, -p.b / J,     0,
           0,          1,           0;
    Mc.topLeftCorner(3, 3) = Ac;
    Mc(0, 3) = 1.0 / p.L;
    Mc(1, 4) = -1.0 / J;
    const MatrixXd Md = expm(Mc * p.Ts);

    const MatrixXd A = Md.topLeftCorner(nx, nx);
    const VectorXd B = Md.block(0, nx, nx, 1);
    const VectorXd Bd = Md.block(0, nx + 1, nx, 1);

    std::vector<MatrixXd> pw(Np);
    pw[0] = MatrixXd::Identity(nx, nx);
    for (int k = 1; k < Np; ++k) pw[k] = A * pw[k - 1];

    MatrixXd Sx(nx * Np, nx), SuFull = MatrixXd::Zero(nx * Np, Np);
    VectorXd Su0(nx * Np), Sd(nx * Np);
    MatrixXd Asum = MatrixXd::Zero(nx, nx);
    for (int kk = 1; kk <= Np; ++kk) {
        const int r = (kk - 1) * nx;
        Sx.block(r, 0, nx, nx) = A * pw[kk - 1];
        Su0.segment(r, nx) = pw[kk - 1] * B;
        Asum += pw[kk - 1];
        Sd.segment(r, nx) = Asum * Bd;
        for (int jj = 1; jj < kk; ++jj) SuFull.block(r, jj - 1, nx, 1) = pw[kk - jj - 1] * B;
    }

    MatrixXd Tu = MatrixXd::Zero(Np, Nc);
    Tu.topLeftCorner(Nc, Nc).setIdentity();
    for (int i = Nc; i < Np; ++i) Tu(i, Nc - 1) = 1.0;

    const MatrixXd Su = SuFull * Tu;

    MatrixXd Csel = MatrixXd::Zero(Np, nx * Np);
    MatrixXd Qbar = MatrixXd::Zero(nx * Np, nx * Np);
    for (int k = 0; k < Np; ++k) {
        Csel(k, nx * k) = 1.0;
        for (int i = 0; i < nx; ++i) Qbar(nx * k + i, nx * k + i) = p.Qdiag(i);
    }
    const MatrixXd Rwbar = p.Rw * MatrixXd::Identity(Nc, Nc);

    MatrixXd Dd = MatrixXd::Identity(Nc, Nc);
    for (int i = 1; i < Nc; ++i) Dd(i, i - 1) = -1.0;
    VectorXd e1 = VectorXd::Zero(Nc);
    e1(0) = 1.0;

    Matrices m;
    m.umx = p.umx;

    const MatrixXd SuT_Qbar = Su.transpose() * Qbar;
    const MatrixXd Huu = 2.0 * (SuT_Qbar * Su + Dd.transpose() * Rwbar * Dd);
    m.H = MatrixXd::Zero(Nc + Np, Nc + Np);
    m.H.topLeftCorner(Nc, Nc) = Huu;
    m.H.bottomRightCorner(Np, Np) = 2.0 * p.Wslack * MatrixXd::Identity(Np, Np);
    m.H = 0.5 * (m.H + m.H.transpose());
    m.H += 1e-9 * m.H.diagonal().maxCoeff() * MatrixXd::Identity(Nc + Np, Nc + Np);

    m.Fx = 2.0 * SuT_Qbar * Sx;
    m.Fu0 = 2.0 * SuT_Qbar * Su0 - 2.0 * Dd.transpose() * Rwbar * e1;
    m.Fd = 2.0 * SuT_Qbar * Sd;
    m.Fr = -2.0 * SuT_Qbar;

    const MatrixXd CselSu = Csel * Su;
    const MatrixXd CselSx = Csel * Sx;
    const VectorXd CselSu0 = Csel * Su0;
    const VectorXd CselSd = Csel * Sd;
    const MatrixXd INp = MatrixXd::Identity(Np, Np);
    const MatrixXd INc = MatrixXd::Identity(Nc, Nc);

    const int MM = 3 * Np + 2 * Nc;
    m.Aineq = MatrixXd::Zero(MM, Nc + Np);
    m.Ex = MatrixXd::Zero(MM, nx);
    m.Eu0 = VectorXd::Zero(MM);
    m.Ed = VectorXd::Zero(MM);
    m.bconst = VectorXd::Zero(MM);

    int r = 0;
    m.Aineq.block(r, 0, Np, Nc) = CselSu;
    m.Aineq.block(r, Nc, Np, Np) = -INp;
    m.Ex.block(r, 0, Np, nx) = -CselSx;
    m.Eu0.segment(r, Np) = -CselSu0;
    m.Ed.segment(r, Np) = -CselSd;
    m.bconst.segment(r, Np).setConstant(p.imx);
    r += Np;
    m.Aineq.block(r, 0, Np, Nc) = -CselSu;
    m.Aineq.block(r, Nc, Np, Np) = -INp;
    m.Ex.block(r, 0, Np, nx) = CselSx;
    m.Eu0.segment(r, Np) = CselSu0;
    m.Ed.segment(r, Np) = CselSd;
    m.bconst.segment(r, Np).setConstant(p.imx);
    r += Np;
    m.Aineq.block(r, 0, Nc, Nc) = INc;
    m.bconst.segment(r, Nc).setConstant(p.umx);
    r += Nc;
    m.Aineq.block(r, 0, Nc, Nc) = -INc;
    m.bconst.segment(r, Nc).setConstant(p.umx);
    r += Nc;
    m.Aineq.block(r, Nc, Np, Np) = -INp;

    m.Aaug = MatrixXd::Zero(nx + 1, nx + 1);
    m.Aaug.topLeftCorner(nx, nx) = A;
    m.Aaug.block(0, nx, nx, 1) = Bd;
    m.Aaug(nx, nx) = 1.0;
    m.Baug = VectorXd::Zero(nx + 1);
    m.Baug.head(nx) = B;
    m.Caug = MatrixXd::Zero(2, nx + 1);
    m.Caug(0, 0) = 1.0;
    m.Caug(1, 2) = 1.0;

    const MatrixXd Qn = MatrixXd(p.Qndiag.asDiagonal());
    const MatrixXd Rn = MatrixXd(p.Rndiag.asDiagonal());
    const MatrixXd P = filterRiccati(m.Aaug, m.Caug, Qn, Rn);
    const MatrixXd S = m.Caug * P * m.Caug.transpose() + Rn;
    m.Lgain = m.Aaug * P * m.Caug.transpose() * S.inverse();

    if (!m.H.allFinite() || !m.Lgain.allFinite())
        throw std::runtime_error("matrices_preparation: non-finite result");

    return m;
}

}


