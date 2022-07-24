#include <iostream>
#include <Eigen/Dense>

int main(int argc, char **argv)
{
  double raw[] = {1, 0, 0, 0, 0, 0};
  Eigen::Map<const Eigen::Matrix<double, 2, 3>> A(raw);
  // Eigen::Matrix<double, 2, 3> A = Eigen::MatrixXd::Random(2, 3);
  std::cout << "A:\n" << A << std::endl;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = std::numeric_limits<double>::epsilon() * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs()(0);

  Eigen::Matrix<double, 3, 2> Ap = svd.matrixV() * (
    svd.singularValues().array().abs() > tolerance
  ).select(
    svd.singularValues().array().inverse(), 0
  ).matrix().asDiagonal() * svd.matrixU().adjoint();
  std::cout << "A+:\n" << Ap << std::endl;

  Eigen::MatrixXd P = Ap * A;
  std::cout << "range(A^T) projector:\n" << P << std::endl;
  Eigen::MatrixXd P2 = svd.matrixV() * svd.matrixV().adjoint();
  std::cout << "range(A^T) projector (wrong alternative computation):\n" << P2 << std::endl;
  return 0;
}
