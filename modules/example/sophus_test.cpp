#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/se3.hpp"
using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1))
                   .toRotationMatrix();  //申请一个旋转向量，然后转为旋转矩阵
  //或者四元数
  Quaterniond q(R);  //将矩阵转为四元数
  //转为SO3，以下相等
  Sophus::SO3d SO3d_R(R);  //将旋转矩阵转为SO3；
  Sophus::SO3d SO3d_q(q);  //将四元数转为SO3；
  cout << "they are equal:" << endl;
  cout << "SO3d_R from R:\n"
       << SO3d_R.matrix() << endl;  //将SO3d_R转为矩阵输出；
  cout << "SO3d_q from q:\n"
       << SO3d_q.matrix() << endl;  //将SO3d_q转为矩阵输出；

  //使用log映射SO3为李代数
  Vector3d so3 = SO3d_q.log();
  cout << "so3=" << so3.transpose() << endl;  //转置输出
  // hat为向量到反对称矩阵，例如so3->SO3
  cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
  // vee,为反对成矩阵到向量
  cout << "so3 hat to vee:"
       << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

  //增量扰动模型的更新
  Vector3d update_so3(1e-4, 0, 0);  //假设更新量
  Sophus::SO3d SO3_updated =
      Sophus::SO3d::exp(update_so3) * SO3d_R;  // so3->SO3,使用exp函数；
  cout << "SO3_updated=\n" << SO3_updated.matrix() << endl;
  cout << "********************************************************************"
          "**************************************************"
       << endl;
  //对SE(3)操作大同小异；
  Vector3d t(1, 0, 0);         //沿X轴平移1；
  Sophus::SE3d SE3d_Rt(R, t);  //通过旋转矩阵和平移向量组成SE3；
  Sophus::SE3d SE3d_qt(q, t);  //通过四元数和平移向量组成SE3；
  cout << "SE3 from R,t=\n" << SE3d_Rt.matrix() << endl;
  cout << "SE3 from q,t=\n" << SE3d_qt.matrix() << endl;
  //因为se(3)是一个6维向量，因此typedef一个;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3d_qt.log();
  cout << "se3=" << se3.transpose() << endl;

  //同样测试hat和vee;
  cout << "hat=\n" << Sophus::SE3d::hat(se3) << endl;
  cout << "vee=" << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose()
       << endl;

  //测试扰动模型;
  Vector6d update_se3;   //更新量;
  update_se3.setZero();  //初始化为0；
  update_se3(0, 0) = 1e-4;
  Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3d_Rt;
  cout << "SE3_updated=\n" << SE3_updated.matrix() << endl;
  return 0;
}
