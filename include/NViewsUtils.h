#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

// include types
#include "NViewsTypes.h"

namespace NViewsTrian{
                        
// check constraints                 
double checkConstraints(const Eigen::VectorXd & sol, 
                        std::vector<Constr2View> & constr, 
                        Eigen::VectorXd & val_constr,
                        double & max_constr_val, 
                        double & sq_constr_val);   
                                             
// solve linear system minimum norm
double solveLinearSystemMinNorm(const Eigen::MatrixXd & A, 
                                const Eigen::VectorXd & b,
                                Eigen::VectorXd & sol);
// triangulate point
double triangulateNPoint(const std::vector<Eigen::Matrix<double, 3, 4>> & proj_s,
                         const std::vector<Eigen::Vector3d> & obs_s, 
                         Eigen::Vector3d & P_3d, 
                         Eigen::VectorXd & depths);
                         
                         
double triangulateNPoint(const std::vector<Eigen::Matrix4d> & proj_s,
                         const std::vector<Eigen::Vector3d> & obs_s, 
                         Eigen::Vector3d & P_3d, 
                         Eigen::VectorXd & depths);

double solveLinearSystemMinNormIt(const Eigen::MatrixXd & A, 
                                  const Eigen::VectorXd & b,
                                  Eigen::VectorXd & sol);                      
}  // end of namespace NViewsTrian
