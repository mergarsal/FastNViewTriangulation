#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

// include types
#include "NViewsTypes.h"


namespace NViewsTrian
{

        /* Struct for the result */
        struct NCertRes
        {
                double min_eig = -10;
                double error_mult = -10; 
                Eigen::VectorXd mult; 
                Eigen::MatrixXd Hess; 
                
                double time_mult = 10; 
                double time_hess = 10; 
                
                NCertRes(){};
        };  // end of struct result
        
        
        class NViewCertClass
        {
                public:
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
                        
                        NViewCertClass(const int M, 
                                       const std::vector<Constr2View>& constr, 
                                       bool debug = true):
                                        M_(2*M), constr_red_(constr), debug_(debug){};
                        ~NViewCertClass(){};
                        
                        /* Compute multipliers */
                                           
                        double computeMult(const Eigen::VectorXd & sol, 
                                           const Eigen::MatrixXd & C, 
                                           Eigen::VectorXd & sol_mult);                                        
                                                      
                        /* Form Hessian and compute minimum eigenvalue */
                        double computeHessian(const double cost_sol, 
                                              const Eigen::VectorXd & mult, 
                                              Eigen::MatrixXd & Hess);  
                                                               
                        /* Check optimality of solution */                        
                        NCertRes checkOptimality(const Eigen::VectorXd & sol, 
                                                 const Eigen::MatrixXd & C);
                        
                        /* Print results */
                        void printResult(const NCertRes & res); 
                        
                private:
                        std::vector<Constr2View> constr_red_;  // constraints 
                        int M_;  // 2 * number cameras
                        bool debug_;  // debug flag
                
        };  // end of main class



}  // end of namespace
