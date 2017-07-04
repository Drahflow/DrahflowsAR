#ifndef KALMAN_UKF_HPP
#define KALMAN_UKF_HPP

#include <Eigen/Dense>
#include <functional>
#include <iostream>

// Unscented Kalman Filter, as per Wikipedia
//
// n: Number of state variables
// T: Datatype used for calculations. Must be supported by Eigen, better use double or better.
template<int n, typename T = double>
class UnscentedKalmanFilter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    template<int r, int c> using Matrix = Eigen::Matrix<T, r, c>;

    // Dynamically estimated state of the system
    Matrix<n, 1> x;

    // Dynamically estimated covariance of the system
    Matrix<n, n> P;

  public:
    UnscentedKalmanFilter(const Matrix<n, 1> &x0, const Matrix<n, n> &P0): x(x0), P(P0) { }

    typedef struct {
      Matrix<n, 1> x;
      Matrix<n, n> P;
    } State;

    State state() const {
      return State{x, P};
    }

    // w: Mean of process noise.
    // Q: Covariance of process noise.
    // F: The state propagation function.
    template<typename F>
    void predict(const Matrix<n, 1> &w, const Matrix<n, n> &Q, const F &f) {
      Matrix<2*n, 1> x_a; x_a << x, w;
      Matrix<2*n, 2*n> P_a; P_a << P, P.Zero(), Q.Zero(), Q;

      constexpr int L = 2 * n; // aliasing to follow Wikipedia more closely
      constexpr T alpha = 1e-3;
      constexpr T kappa = 3 - L;
      constexpr T beta = 2;
      constexpr T lambda = alpha * alpha * (L + kappa) - L;

      Matrix<2*n, 2*n> covarianceRoot = ((L + lambda) * P_a).llt().matrixL();

      Matrix<2*n, 1> xi[2*L + 1];
      xi[0] = x_a;
      for(unsigned int i = 1; i <= L; ++i) {
        xi[i] = x_a + covarianceRoot.col(i - 1);
      }
      for(unsigned int i = L + 1; i <= 2*L; ++i) {
        xi[i] = x_a - covarianceRoot.col(i - L - 1);
      }

      Matrix<n, 1> xi_k[2*L + 1];
      for(unsigned int i = 0; i < 2 * L + 1; ++i) {
        xi_k[i] = f(xi[i].template block<n, 1>(0, 0)) + xi[i].template block<n, 1>(n, 0);
      }

      const T W_s0 = lambda / (L + lambda);
      Matrix<n, 1> x_k = W_s0 * xi_k[0];
      for(unsigned int i = 1; i < 2 * L + 1; ++i) {
        const T W_si = 1 / (2 * (L + lambda));
        x_k += W_si * xi_k[i];
      }

      const T W_c0 = lambda / (L + lambda) + (1 - alpha * alpha + beta);
      Matrix<n, n> P_k = W_c0 * (xi_k[0] - x_k) * (xi_k[0] - x_k).transpose();
      for(unsigned int i = 1; i < 2 * L + 1; ++i) {
        const T W_ci = 1 / (2 * (L + lambda));
        P_k += W_ci * (xi_k[i] - x_k) * (xi_k[i] - x_k).transpose();
      }

      x = x_k;
      P = P_k;
    }

    // v: Mean of observation noise.
    // R: Covariance of observation noise.
    // H: Observation function.
    // z: Actual data.
    template<int m, typename H>
    void update(const Matrix<m, 1> &v, const Matrix<m, m> &R, const H &h, const Matrix<m, 1> &z) {
      Matrix<n + m, 1> x_a; x_a << x, v;
      Matrix<n + m, n + m> P_a; P_a << P, Matrix<n, m>::Zero(), Matrix<m, n>::Zero(), R;

      constexpr int L = n + m; // aliasing to follow Wikipedia more closely
      constexpr T alpha = 1e-3;
      constexpr T kappa = 3 - L;
      constexpr T beta = 2;
      constexpr T lambda = alpha * alpha * (L + kappa) - L;

      Matrix<n + m, n + m> covarianceRoot = ((L + lambda) * P_a).llt().matrixL();

      Matrix<n + m, 1> xi[2*L + 1];
      xi[0] = x_a;
      for(unsigned int i = 1; i <= L; ++i) {
        xi[i] = x_a + covarianceRoot.col(i - 1);
      }
      for(unsigned int i = L + 1; i <= 2*L; ++i) {
        xi[i] = x_a - covarianceRoot.col(i - L - 1);
      }

      Matrix<m, 1> gamma_k[2*L + 1];
      for(unsigned int i = 0; i < 2 * L + 1; ++i) {
        gamma_k[i] = h(xi[i].template block<n, 1>(0, 0)) + xi[i].template block<m, 1>(n, 0);
      }

      const T W_s0 = lambda / (L + lambda);
      Matrix<m, 1> z_k = W_s0 * gamma_k[0];
      for(unsigned int i = 1; i < 2 * L + 1; ++i) {
        const T W_si = 1 / (2 * (L + lambda));
        z_k += W_si * gamma_k[i];
      }

      const T W_c0 = lambda / (L + lambda) + (1 - alpha * alpha + beta);
      Matrix<m, m> P_zzk = W_c0 * (gamma_k[0] - z_k) * (gamma_k[0] - z_k).transpose();
      for(unsigned int i = 1; i < 2 * L + 1; ++i) {
        const T W_ci = 1 / (2 * (L + lambda));
        P_zzk += W_ci * (gamma_k[i] - z_k) * (gamma_k[i] - z_k).transpose();
      }

      // W_c0 = ... as before
      Matrix<n, m> P_xzk = W_c0 * (xi[0].template block<n, 1>(0, 0) - x) * (gamma_k[0] - z_k).transpose();
      for(unsigned int i = 1; i < 2 * L + 1; ++i) {
        const T W_ci = 1 / (2 * (L + lambda));
        P_xzk += W_ci * (xi[i].template block<n, 1>(0, 0) - x) * (gamma_k[i] - z_k).transpose();
      }

      Matrix<n, m> K_k = P_xzk * P_zzk.inverse();

      x += K_k * (z - z_k);
      P -= K_k * P_zzk * K_k.transpose();
    }
};

// Takes a function f'(x) and returns a kalman filter propagation function based on it
template<typename R, typename T, typename X>
std::function<R(const X&)> EulerPropagator(const std::function<R(const X&)> &f, const T &dt) {
  return [&f, &dt](const X &x) {
    return x + dt * f(x);
  };
}

// Takes a function f'(x) and returns a kalman filter propagation function based on it
template<typename R, typename T, typename X>
std::function<R(const X&)> RungeKuttaPropagator(const std::function<R(const X&)> &f, const T &dt) {
  return [&f, &dt](const X &x) {
    auto k1 = f(x);
    auto k2 = f(x + dt/2 * k1);
    auto k3 = f(x + dt/2 * k2);
    auto k4 = f(x + dt * k3);
    return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
  };
}

#ifdef TESTING
// trivial sanity test with falling ball example
// run via
// g++ -O4 -W -Wall -Wextra -std=c++17 -I/usr/include/eigen3 -DTESTING -o ukf -x c++ ukf.hpp && valgrind ./ukf

#include <iostream>

int main(void) {
  using namespace Eigen;
  typedef double T;

  Matrix<T, 2, 1> x0;
  x0 << 10, 0;

  Matrix<T, 2, 1> processError;
  processError << 0.1, 0.1;
  Matrix<T, 2, 2> processCovariance;
  processCovariance << 0.1, 0.1, 0.1, 0.1;

  Matrix<T, 1, 1> measureError;
  measureError << 0.5;
  Matrix<T, 1, 1> measureCovariance;
  measureCovariance << 0.5;

  UnscentedKalmanFilter<2, T> ukf(x0, processCovariance);

  std::function<Matrix<T, 2, 1>(const Matrix<T, 2, 1> &)> f = [](const Matrix<T, 2, 1> &x) -> auto {
    Matrix<T, 2, 1> xdt;
    xdt(0) = x(1);
    xdt(1) = -9.81;
    return xdt;
  };

  constexpr T dt = 0.01;
  for(T t = 0; t <= 1 + 1e-6; t += dt) {
    std::cout << "t: " << t << "  x: " << ukf.state().x.transpose() << std::endl << "  P: " << ukf.state().P << std::endl;
    ukf.predict(processError, processCovariance,
        RungeKuttaPropagator(f, dt));
    ukf.update<1>(measureError, measureCovariance,
        [](const auto &x) -> auto { return x.row(0); }, Matrix<T, 1, 1>{10 - 0.5 * t * t * 9.81});
  }

  return 0;
}
#endif

#endif
