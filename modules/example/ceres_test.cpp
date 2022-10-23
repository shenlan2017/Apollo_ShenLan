#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>

class ExpResidual {
 public:
  ExpResidual(double x, double y) : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T* m, const T* c, T* residual) const {
    residual[0] = T(y_) - ceres::exp(m[0] * T(x_) + c[0]);
    return true;
  }

  const double x_;
  const double y_;
};

int main() {
  double m = 0.0, c = 0.0;
  cv::RNG rng;
  double w_sigma = 0.2;

  ceres::Problem problem;
  for (int i = 0; i < 100; ++i) {
    double y_with_noise =
        std::exp(0.3 * i / 100.0 + 0.1) + rng.gaussian(w_sigma * w_sigma);

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ExpResidual, 1, 1, 1>(
            new ExpResidual(i / 100.0, y_with_noise));
    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), &m, &c);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;
  std::cout << "m = " << m << " c = " << c << std::endl;
  return 0;
}
