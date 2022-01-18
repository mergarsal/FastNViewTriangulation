#pragma once 

#include <cmath>
#include <cstdio>
#include <iostream>

#include <Eigen/Core>
#include "ceres/ceres.h"


namespace CeresSolver
{

 Eigen::Vector3d Triangulate(std::vector<std::pair
        <Eigen::Matrix<double, 3, 4>,Eigen::Vector2d>>& datas, 
        Eigen::Vector3d & init_x) ;      

                        
};  // end of namespace ceresSolver

