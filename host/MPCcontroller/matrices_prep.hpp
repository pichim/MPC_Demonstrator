#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace mpc_prep {

struct Params {
    double Ts = 1e-3;
    int N = 20;

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
    if (!converged) throw std::runtime_error("Kalman Riccati equation did not converge");
    return Hk;
}

inline Matrices compute(const Params &p)
{
    if (p.N < 2 || p.Ts <= 0 || p.L <= 0 || p.R <= 0 || p.k_m == 0 || p.J_M + p.J_L <= 0 ||
        p.Rw <= 0 || p.Wslack <= 0 || p.imx <= 0 || p.umx <= 0 || p.b < 0)
        throw std::invalid_argument("matrices_preparation: invalid parameter");

    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const int N = p.N;
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

    std::vector<MatrixXd> pw(N);
    pw[0] = MatrixXd::Identity(nx, nx);
    for (int k = 1; k < N; ++k) pw[k] = A * pw[k - 1];

    MatrixXd Sx(nx * N, nx), Su = MatrixXd::Zero(nx * N, N);
    VectorXd Su0(nx * N), Sd(nx * N);
    MatrixXd Asum = MatrixXd::Zero(nx, nx);
    for (int kk = 1; kk <= N; ++kk) {
        const int r = (kk - 1) * nx;
        Sx.block(r, 0, nx, nx) = A * pw[kk - 1];
        Su0.segment(r, nx) = pw[kk - 1] * B;
        Asum += pw[kk - 1];
        Sd.segment(r, nx) = Asum * Bd;
        for (int jj = 1; jj < kk; ++jj) Su.block(r, jj - 1, nx, 1) = pw[kk - jj - 1] * B;
    }

    MatrixXd Csel = MatrixXd::Zero(N, nx * N);
    MatrixXd Qbar = MatrixXd::Zero(nx * N, nx * N);
    for (int k = 0; k < N; ++k) {
        Csel(k, nx * k) = 1.0;
        for (int i = 0; i < nx; ++i) Qbar(nx * k + i, nx * k + i) = p.Qdiag(i);
    }
    const MatrixXd Rwbar = p.Rw * MatrixXd::Identity(N, N);

    MatrixXd Dd = MatrixXd::Identity(N, N);
    for (int i = 1; i < N; ++i) Dd(i, i - 1) = -1.0;
    VectorXd e1 = VectorXd::Zero(N);
    e1(0) = 1.0;

    Matrices m;
    m.umx = p.umx;

    const MatrixXd SuT_Qbar = Su.transpose() * Qbar;
    const MatrixXd Huu = 2.0 * (SuT_Qbar * Su + Dd.transpose() * Rwbar * Dd);
    m.H = MatrixXd::Zero(2 * N, 2 * N);
    m.H.topLeftCorner(N, N) = Huu;
    m.H.bottomRightCorner(N, N) = 2.0 * p.Wslack * MatrixXd::Identity(N, N);
    m.H = 0.5 * (m.H + m.H.transpose());
    m.H += 1e-9 * m.H.diagonal().maxCoeff() * MatrixXd::Identity(2 * N, 2 * N);

    m.Fx = 2.0 * SuT_Qbar * Sx;
    m.Fu0 = 2.0 * SuT_Qbar * Su0 - 2.0 * Dd.transpose() * Rwbar * e1;
    m.Fd = 2.0 * SuT_Qbar * Sd;
    m.Fr = -2.0 * SuT_Qbar;

    const MatrixXd CselSu = Csel * Su;
    const MatrixXd CselSx = Csel * Sx;
    const VectorXd CselSu0 = Csel * Su0;
    const VectorXd CselSd = Csel * Sd;
    const MatrixXd IN = MatrixXd::Identity(N, N);
    MatrixXd IboxU = MatrixXd::Zero(N - 1, N);
    IboxU.leftCols(N - 1).setIdentity();

    const int MM = 5 * N - 2;
    m.Aineq = MatrixXd::Zero(MM, 2 * N);
    m.Ex = MatrixXd::Zero(MM, nx);
    m.Eu0 = VectorXd::Zero(MM);
    m.Ed = VectorXd::Zero(MM);
    m.bconst = VectorXd::Zero(MM);

    int r = 0;
    m.Aineq.block(r, 0, N, N) = CselSu;
    m.Aineq.block(r, N, N, N) = -IN;
    m.Ex.block(r, 0, N, nx) = -CselSx;
    m.Eu0.segment(r, N) = -CselSu0;
    m.Ed.segment(r, N) = -CselSd;
    m.bconst.segment(r, N).setConstant(p.imx);
    r += N;
    m.Aineq.block(r, 0, N, N) = -CselSu;
    m.Aineq.block(r, N, N, N) = -IN;
    m.Ex.block(r, 0, N, nx) = CselSx;
    m.Eu0.segment(r, N) = CselSu0;
    m.Ed.segment(r, N) = CselSd;
    m.bconst.segment(r, N).setConstant(p.imx);
    r += N;
    m.Aineq.block(r, 0, N - 1, N) = IboxU;
    m.bconst.segment(r, N - 1).setConstant(p.umx);
    r += N - 1;
    m.Aineq.block(r, 0, N - 1, N) = -IboxU;
    m.bconst.segment(r, N - 1).setConstant(p.umx);
    r += N - 1;
    m.Aineq.block(r, N, N, N) = -IN;

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



