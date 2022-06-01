#include <articulated/franka/articulated_franka.h>

int main(int argc, char **argv)
{
  double raw[] = {1, 0, 0, 1, 0, 1};
  Eigen::Map<const Eigen::MatrixXd> A(raw, 2, 3);
  // Eigen::Matrix<double, 2, 3> Ar;
  // Ar << 1, 0, 0, 0, 1, 1;
  // Eigen::MatrixXd A(2, 3);
  // A = Ar;
  std::cout << "A:\n" << A << std::endl;

  Eigen::MatrixXd Ap = A.completeOrthogonalDecomposition().pseudoInverse();
  std::cout << "A+:\n" << Ap << std::endl;

  Eigen::MatrixXd N = Ap * A;
  std::cout << "N:\n" << N << std::endl;
  return 0;
}
