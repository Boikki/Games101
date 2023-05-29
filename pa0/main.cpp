#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include "color.h"

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;
    printf(RED_HL("=======\n"));

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    std::cout << v[1] << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    printf(GREEN_HL("Example of dot multiply \n"));
    std::cout << w.dot(v) << std::endl;
    printf(RED_HL("Example of cross multiply \n"));
    std::cout << w.cross(v) << std::endl;
    Eigen::Vector2f v1(2.0f,3.0f);
    Eigen::Vector2f w1(0.0f,0.0f);
    std::cout << w1.cross(v1) << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output matrix i\n";
    std::cout << i << std::endl;
    printf(BLUE_HL("matrix j\n"));
    std::cout << j << std::endl;
    // matrix add i + j
    printf(BLUE_HL("matrix add\n"));
    std::cout << i + j << std::endl;
    // matrix scalar multiply i * 2.0
    printf(BLUE_HL("matrix multiply 2\n"));
    std::cout << i * 2.0 << std::endl;
    // matrix multiply i * j
    printf(BLUE_HL("matrix multiply i * j\n"));
    std::cout << i * j << std::endl;
    // matrix multiply vector i * v

    Eigen::Vector3f x(2.0f, 1.0f, 1.0f);
    float m = cos(45.0 / 180.0 * acos(-1));
    float n = sin(45.0 / 180.0 * acos(-1));
    Eigen::Matrix3f k;
    k << m, -n, 1, m, n, 2, 0, 0, 1;
    printf(BLUE_HL("matrix \n"));
    std::cout << x << std::endl;
    printf(BLUE_HL("matrix multiply \n"));
    std::cout << k * x << std::endl;
    
    return 0;
}
