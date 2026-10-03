#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>

namespace mpc_fixed {
    constexpr int N   = 20;
    constexpr int NX  = 3;
    constexpr int NXA = 4;
    constexpr int NN  = 2*N;
    constexpr int MM  = 5*N - 2;
    constexpr int NR  = 3*N;
}

class MpcController {
public:
    static constexpr int N   = mpc_fixed::N;
    static constexpr int NX  = mpc_fixed::NX;
    static constexpr int NXA = mpc_fixed::NXA;
    static constexpr int NN  = mpc_fixed::NN;
    static constexpr int MM  = mpc_fixed::MM;
    static constexpr int NR  = mpc_fixed::NR;

    using MatU   = Eigen::Matrix<double, N, N>;
    using VecS   = Eigen::Matrix<double, N, 1>;
    using VecN   = Eigen::Matrix<double, NN, 1>;
    using VecM   = Eigen::Matrix<double, MM, 1>;
    using VecXA  = Eigen::Matrix<double, NXA, 1>;
    using VecX   = Eigen::Matrix<double, NX, 1>;
    using VecP   = Eigen::Matrix<double, NX + 3, 1>;   
    using MatFf  = Eigen::Matrix<double, N, NX + 3>;
    using MatEb  = Eigen::Matrix<double, MM, NX + 2>;  
    using MatAcl = Eigen::Matrix<double, NXA, NXA>;
    using MatLg  = Eigen::Matrix<double, NXA, 2>;

   
    static constexpr int    MAX_IT      = 50;     
    static constexpr double TOL_FEAS    = 1e-4;   
    static constexpr double TOL_MU      = 1e-4;   
    static constexpr double TAU         = 0.995;
    static constexpr double W_INIT_MIN  = 1e-1;   
    static constexpr double W_INIT_MU   = 1e-1;   
    static constexpr double REG         = 1e-6;

    MpcController(const Eigen::MatrixXd& H_, const Eigen::MatrixXd& Aineq_,
                  const Eigen::MatrixXd& Fx_, const Eigen::VectorXd& Fu0_,
                  const Eigen::VectorXd& Fd_, const Eigen::MatrixXd& Fr_,
                  const Eigen::MatrixXd& Ex_, const Eigen::VectorXd& Eu0_,
                  const Eigen::VectorXd& Ed_, const Eigen::VectorXd& bconst_,
                  double umx_,
                  const Eigen::MatrixXd& Aaug_, const Eigen::VectorXd& Baug_,
                  const Eigen::MatrixXd& Caug_, const Eigen::MatrixXd& Lgain_)
        : umx(umx_)
    {
        auto check = [](const char* name, int r, int c, int er, int ec) {
            if (r != er || c != ec)
                throw std::invalid_argument(std::string("MpcController: ") + name +
                    " has wrong size for compiled-in N=" + std::to_string(N));
        };
        check("H", (int)H_.rows(), (int)H_.cols(), NN, NN);
        check("Aineq", (int)Aineq_.rows(), (int)Aineq_.cols(), MM, NN);
        check("Fx", (int)Fx_.rows(), (int)Fx_.cols(), N, NX);
        check("Fu0", (int)Fu0_.size(), 1, N, 1);
        check("Fd", (int)Fd_.size(), 1, N, 1);
        check("Fr", (int)Fr_.rows(), (int)Fr_.cols(), N, NR);
        check("Ex", (int)Ex_.rows(), (int)Ex_.cols(), MM, NX);
        check("Eu0", (int)Eu0_.size(), 1, MM, 1);
        check("Ed", (int)Ed_.size(), 1, MM, 1);
        check("bconst", (int)bconst_.size(), 1, MM, 1);
        check("Aaug", (int)Aaug_.rows(), (int)Aaug_.cols(), NXA, NXA);
        check("Baug", (int)Baug_.size(), 1, NXA, 1);
        check("Caug", (int)Caug_.rows(), (int)Caug_.cols(), 2, NXA);
        check("Lgain", (int)Lgain_.rows(), (int)Lgain_.cols(), NXA, 2);

        Eigen::MatrixXd H = H_, A = Aineq_;
        double twoWslack = H(N, N);
        if (twoWslack > 0.0) {
            double wslack = twoWslack / 2.0;
            double sscale = std::sqrt(wslack);
            H.block(N, N, N, N) /= wslack;
            A.block(0, N, MM, N) /= sscale;
        }

        const double eps = 1e-12;
        if (H.topRightCorner(N, N).cwiseAbs().maxCoeff() > eps)
            throw std::invalid_argument("MpcController: H couples u and slack (unsupported)");
        Eigen::MatrixXd Hss = H.bottomRightCorner(N, N);
        Eigen::MatrixXd HssD = Hss.diagonal().asDiagonal();
        if ((Hss - HssD).cwiseAbs().maxCoeff() > eps)
            throw std::invalid_argument("MpcController: slack block of H not diagonal (unsupported)");

        Eigen::MatrixXd Hu_ = H.topLeftCorner(N, N);
        Hu = 0.5 * (Hu_ + Hu_.transpose());
        hs = Hss.diagonal();
        HuReg = Hu + REG * MatU::Identity();
        hsReg = hs.array() + REG;

        rowPtr.assign(MM + 1, 0);
        sCol.assign(MM, -1);
        sVal.assign(MM, 0.0);
        for (int i = 0; i < MM; ++i) {
            rowPtr[i] = (int)uIdx.size();
            for (int j = 0; j < N; ++j)                       
                if (std::abs(A(i, j)) > eps) { uIdx.push_back(j); uVal.push_back(A(i, j)); }
            int ns = 0;
            for (int j = N; j < NN; ++j)
                if (std::abs(A(i, j)) > eps) {
                    if (++ns > 1)
                        throw std::invalid_argument("MpcController: a constraint row has >1 slack entry (unsupported)");
                    sCol[i] = j - N; sVal[i] = A(i, j);
                }
        }
        rowPtr[MM] = (int)uIdx.size();

        VecS FrRef = VecS::Zero();                           
        for (int k = 0; k < N; ++k) FrRef += Fr_.col(3*k + 2);
        Ff.template leftCols<NX>() = Fx_;
        Ff.col(NX)     = Fu0_;
        Ff.col(NX + 1) = Fd_;
        Ff.col(NX + 2) = FrRef;
        Eb.template leftCols<NX>() = Ex_;
        Eb.col(NX)     = Eu0_;
        Eb.col(NX + 1) = Ed_;
        bconst = bconst_;

        Lgain  = Lgain_;
        Acl    = Aaug_ - Lgain_ * Caug_;
        Baug   = Baug_;

        xhat.setZero();
        z_prev.setZero();
        u_prev_valid = 0.0;
        fail_count = 0;
    }

    struct Result {
        double u_apply;
        VecXA xhat;
        int status;
        int iters;
        int fault;
    };

    Result step(const Eigen::Vector2d& xmeas, double theta_ref) {
        Result out;
        out.status = 0; out.iters = 0; out.fault = 0;

        if (!xhat.allFinite()) xhat.setZero();
        if (!z_prev.allFinite()) z_prev.setZero();
        if (!std::isfinite(u_prev_valid)) u_prev_valid = 0.0;

        if (!(std::isfinite(xmeas(0)) && std::isfinite(xmeas(1)) && std::isfinite(theta_ref))) {
            fallback(out);
            out.xhat = xhat;
            return out;
        }

        VecP p;
        p << xhat.template head<NX>(), u_prev_valid, xhat(NX), theta_ref;

        VecN f;
        f.template head<N>() = Ff * p;
        f.template tail<N>().setZero();
        VecM bineq = bconst + Eb * p.template head<NX + 2>();

        if (!f.allFinite() || !bineq.allFinite()) {
            fallback(out);
            out.xhat = xhat;
            return out;
        }

        VecN z0;
        z0.template head<N - 1>() = z_prev.template segment<N - 1>(1);
        z0(N - 1)                 = z_prev(N - 1);
        z0.template segment<N - 1>(N) = z_prev.template segment<N - 1>(N + 1);
        z0(NN - 1)                    = z_prev(NN - 1);

        int status, iters;
        VecN z = qpSolveIPM(f, bineq, z0, status, iters);
        out.iters = iters;

        double u_apply;
        if (status == 1) {
            fail_count = 0;
            u_apply = z(0);
            z_prev = z;
        } else {
            fail_count++;
            if (fail_count >= MAX_CONSEC_FAIL) {
                u_apply = SAFE_U;
                out.fault = 1;
            } else if (!z.allFinite()) {
                u_apply = u_prev_valid;
            } else {
                u_apply = z(0);
                z_prev = z;
            }
        }
        out.status = status;

        u_apply = std::min(std::max(u_apply, -umx), umx);
        u_prev_valid = u_apply;

        VecXA xn = Acl * xhat + Baug * u_apply + Lgain * xmeas;
        if (xn.allFinite()) xhat = xn;

        out.u_apply = u_apply;
        out.xhat = xhat;
        return out;
    }

private:
   
    MatU  Hu, HuReg;
    VecS  hs, hsReg;
    MatFf Ff;
    MatEb Eb;
    VecM  bconst;
    MatAcl Acl;
    VecXA Baug;
    MatLg Lgain;
    double umx;

    std::vector<int>    rowPtr, uIdx, sCol;
    std::vector<double> uVal, sVal;

    VecXA xhat;
    VecN  z_prev;
    double u_prev_valid;
    int fail_count;

    MatU S, B;
    VecS D, Dinv;
    Eigen::LLT<MatU>  llt;
    Eigen::LDLT<MatU> ldlt;
    bool useLDLT = false;

    static constexpr int MAX_CONSEC_FAIL = 5;
    static constexpr double SAFE_U = 0.0;

    void fallback(Result& out) {
        fail_count++;
        double u_apply;
        if (fail_count >= MAX_CONSEC_FAIL) { u_apply = SAFE_U; out.fault = 1; }
        else u_apply = u_prev_valid;
        u_apply = std::min(std::max(u_apply, -umx), umx);
        u_prev_valid = u_apply;
        out.u_apply = u_apply;
        out.status = 0;
    }

    void mulA(const VecN& z, VecM& out) const {
        const double* zp = z.data();
        for (int i = 0; i < MM; ++i) {
            double acc = 0.0;
            for (int k = rowPtr[i]; k < rowPtr[i + 1]; ++k) acc += uVal[k] * zp[uIdx[k]];
            if (sCol[i] >= 0) acc += sVal[i] * zp[N + sCol[i]];
            out(i) = acc;
        }
    }
    void mulAT(const VecM& v, VecN& out) const {
        out.setZero();
        double* op = out.data();
        for (int i = 0; i < MM; ++i) {
            const double vi = v(i);
            for (int k = rowPtr[i]; k < rowPtr[i + 1]; ++k) op[uIdx[k]] += uVal[k] * vi;
            if (sCol[i] >= 0) op[N + sCol[i]] += sVal[i] * vi;
        }
    }

    void factorize(const VecM& Wm) {
        S = HuReg;          
        D = hsReg;
        B.setZero();
        for (int i = 0; i < MM; ++i) {
            const double wi = Wm(i);
            const int k0 = rowPtr[i], k1 = rowPtr[i + 1];
            for (int a = k0; a < k1; ++a) {
                const int p = uIdx[a];
                const double wa = wi * uVal[a];
                for (int b = k0; b <= a; ++b) S(p, uIdx[b]) += wa * uVal[b];   // uIdx[b] <= p
            }
            const int q = sCol[i];
            if (q >= 0) {
                const double ws = wi * sVal[i];
                D(q) += ws * sVal[i];
                for (int a = k0; a < k1; ++a) B(uIdx[a], q) += ws * uVal[a];
            }
        }
        Dinv = D.cwiseInverse();
        MatU Bs = B * Dinv.cwiseSqrt().asDiagonal();
        S.template selfadjointView<Eigen::Lower>().rankUpdate(Bs, -1.0);

        llt.compute(S);
        useLDLT = (llt.info() != Eigen::Success);
        if (useLDLT) ldlt.compute(S);
    }

    VecN solveK(const VecN& r) const {
        const VecS rs = r.template tail<N>();
        const VecS rhs = r.template head<N>() - B * rs.cwiseProduct(Dinv);
        VecS xu = useLDLT ? VecS(ldlt.solve(rhs)) : VecS(llt.solve(rhs));
        VecS xs = (rs - B.transpose() * xu).cwiseProduct(Dinv);
        VecN x;
        x.template head<N>() = xu;
        x.template tail<N>() = xs;
        return x;
    }

    static double stepToBoundary(const VecM& v, const VecM& dv, double tau) {
        double alpha = 1.0;
        for (int i = 0; i < MM; ++i)
            if (dv(i) < -1e-12) { double r = -v(i) / dv(i); if (r < alpha) alpha = r; }
        alpha *= tau;
        return alpha < 0.0 ? 0.0 : (alpha > 1.0 ? 1.0 : alpha);
    }

    VecN qpSolveIPM(const VecN& f, const VecM& b, const VecN& z0, int& status, int& iters) {
        VecN z = z0, zGood = z0;
        VecM Az, rp, dw, dlam, dw_aff, dlam_aff, Adz, tmp;
        VecN rd, ATl, dz, dz_aff, rhs, tmpN;

        mulA(z, Az);
        VecM w   = (b - Az).cwiseMax(W_INIT_MIN);
        VecM lam = (W_INIT_MU * w.cwiseInverse()).cwiseMax(1e-6);   // centred start

        status = 0; iters = 0;

        for (int it = 1; it <= MAX_IT; ++it) {
            iters = it;

            mulA(z, Az);
            mulAT(lam, ATl);
            rd.template head<N>() = Hu * z.template head<N>() + f.template head<N>() + ATl.template head<N>();
            rd.template tail<N>() = hs.cwiseProduct(z.template tail<N>()) + f.template tail<N>() + ATl.template tail<N>();
            rp = Az + w - b;
            const double mu = w.dot(lam) / MM;

            if (rd.template lpNorm<Eigen::Infinity>() < TOL_FEAS &&
                rp.template lpNorm<Eigen::Infinity>() < TOL_FEAS && mu < TOL_MU) {
                status = 1; zGood = z; break;
            }

            const VecM wSafe = w.cwiseMax(1e-12);
            factorize(lam.cwiseQuotient(wSafe));

            const VecM rc_aff = w.cwiseProduct(lam);
            tmp = (lam.cwiseProduct(rp) - rc_aff).cwiseQuotient(wSafe);
            mulAT(tmp, tmpN);
            rhs = -rd - tmpN;
            dz_aff = solveK(rhs);
            if (!dz_aff.allFinite()) break;
            mulA(dz_aff, Adz);
            dw_aff = -rp - Adz;
            dlam_aff = (-rc_aff - lam.cwiseProduct(dw_aff)).cwiseQuotient(wSafe);
            if (!dw_aff.allFinite() || !dlam_aff.allFinite()) break;

            const double ap_aff = stepToBoundary(w, dw_aff, TAU);
            const double ad_aff = stepToBoundary(lam, dlam_aff, TAU);
            const double mu_aff = std::max((w + ap_aff * dw_aff).dot(lam + ad_aff * dlam_aff) / MM, 0.0);

            double sigma = 0.0;
            if (mu > 1e-14) {
                const double ratio = mu_aff / mu;
                sigma = std::min(std::max(ratio * ratio * ratio, 0.0), 1.0);
            }

            const VecM rc = w.cwiseProduct(lam) + dw_aff.cwiseProduct(dlam_aff)
                          - VecM::Constant(sigma * mu);
            tmp = (lam.cwiseProduct(rp) - rc).cwiseQuotient(wSafe);
            mulAT(tmp, tmpN);
            rhs = -rd - tmpN;
            dz = solveK(rhs);
            if (!dz.allFinite()) break;
            mulA(dz, Adz);
            dw = -rp - Adz;
            dlam = (-rc - lam.cwiseProduct(dw)).cwiseQuotient(wSafe);
            if (!dw.allFinite() || !dlam.allFinite()) break;

            const double alpha_p = stepToBoundary(w, dw, TAU);
            const double alpha_d = stepToBoundary(lam, dlam, TAU);

            z += alpha_p * dz;
            w = (w + alpha_p * dw).cwiseMax(1e-12);
            lam = (lam + alpha_d * dlam).cwiseMax(1e-12);

            if (!z.allFinite() || !w.allFinite() || !lam.allFinite()) break;
            zGood = z;
        }
        return zGood;
    }
};


