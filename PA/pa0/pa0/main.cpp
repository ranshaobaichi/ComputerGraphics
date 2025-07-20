#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
using namespace Eigen;
int main(){
    // Basic Example of cpp
    // std::cout << "Example of cpp \n";
    // float a = 1.0, b = 2.0;
    // std::cout << a << std::endl;
    // std::cout << a/b << std::endl;
    // std::cout << std::sqrt(b) << std::endl;
    // std::cout << std::acos(-1) << std::endl;
    // std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // // Example of vector
    // std::cout << "Example of vector \n";
    // // vector definition
    // Eigen::Vector3f v(1.0f,2.0f,3.0f);
    // Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // // vector output
    // std::cout << "Example of output \n";
    // std::cout << v << std::endl;
    // // vector add
    // std::cout << "Example of add \n";
    // std::cout << v + w << std::endl;
    // // vector scalar multiply
    // std::cout << "Example of scalar multiply \n";
    // std::cout << v * 3.0f << std::endl;
    // std::cout << 2.0f * v << std::endl;

    // // Example of matrix
    // std::cout << "Example of matrix \n";
    // // matrix definition
    // Eigen::Matrix3f i,j;
    // i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    // j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // // matrix output
    // std::cout << "Example of output \n";
    // std::cout << i << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v
    
    Vector3f P(2.0f, 1.0f, 1.0f);

    float rotate_angle = 45.0f / 180.0f * acos(-1); // 45 degree to rotate
    float tx = 1.0f, ty = 2.0f;

    Matrix3f transform_matrix = Matrix3f::Identity();
    transform_matrix(0,0) = std::cos(rotate_angle);
    transform_matrix(0,1) = -std::sin(rotate_angle);
    transform_matrix(1,0) = std::sin(rotate_angle);
    transform_matrix(1,1) = std::cos(rotate_angle);
    transform_matrix(0,2) = tx;
    transform_matrix(1,2) = ty;

    Vector3f P_transformed = transform_matrix * P;
    std::cout << "Original Point: \n" << P << std::endl;
    std::cout << "Transformed Point: \n" << P_transformed << std::endl;

    float angle = 45.0f/180.0f * M_PI;
    Eigen::Rotation2Df rot(angle);
    Eigen::Translation2f trans(1.0f, 2.0f);

    Eigen::Affine2f T = trans * rot;
    Eigen::Vector2f P2(2.0f, 1.0f);
    Eigen::Vector2f P2_t = T * P2;
    std::cout << "Transformed Point:\n" << P2_t << std::endl;

    return 0;
}