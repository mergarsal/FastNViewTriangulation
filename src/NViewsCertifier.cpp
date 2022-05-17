#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>

// include types
#include "NViewsTypes.h"

// include utils
#include "NViewsUtils.h"

// include headers
#include "NViewsCertifier.h"

// for eigendecomposition
#include <eigen3/Eigen/Dense>
#include <Eigen/Eigenvalues> 

#include <chrono>  // timer

using namespace std::chrono;

namespace NViewsTrian
{


// second version: faster
double NViewCertClass::computeMult(const Eigen::VectorXd & sol, 
                                   const Eigen::MatrixXd & C, 
                                   Eigen::VectorXd & sol_mult)
{
  
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> svd(C.transpose() * C);
        Eigen::MatrixXd U = svd.eigenvectors();
        Eigen::MatrixXd D = svd.eigenvalues().cwiseAbs().cwiseSqrt().asDiagonal().inverse(); 
        
        Eigen::VectorXd ss = D * U.transpose() * sol; 
        
      
        sol_mult =  2 * C * U * D * ss;
        
        
        double error_m = ( 0.5 * C.transpose() * sol_mult - sol).squaredNorm(); 
        
        return error_m;        
        
        }
  


double NViewCertClass::computeHessian(const double cost_sol, 
                                      const Eigen::VectorXd & mult, 
                                      Eigen::MatrixXd & Hess)                     
{
        auto start_t_mult = high_resolution_clock::now();
        Hess = Eigen::MatrixXd::Identity(M_+1, M_+1); 
        Hess(M_, M_) = - cost_sol;
        int N = constr_red_.size();
        
        for (int i=0; i <N; i++)
        {
                int id_1 = constr_red_[i].id_1; 
                int id_2 = constr_red_[i].id_2;
                
                Hess(M_, M_)               -=       mult(i) * constr_red_[i].b;   
                
                Hess.block<2,2>(id_1, id_2)         -= 0.5 * mult(i) * constr_red_[i].F; 
                // the symmetric part
                Hess.block<2,2>(id_2, id_1)         -= 0.5 * mult(i) * constr_red_[i].F.transpose();  
                
                Hess.block<2,1>(id_1, M_) -= 0.5 * mult(i) * constr_red_[i].Fp2; 
                // symmetry
                Hess.block<1,2>(M_, id_1) -= 0.5 * mult(i) * constr_red_[i].Fp2.transpose();  
                Hess.block<2,1>(id_2, M_) -= 0.5 * mult(i) * constr_red_[i].Fp1; 
                // symmetry
                Hess.block<1,2>(M_, id_2) -= 0.5 * mult(i) * constr_red_[i].Fp1.transpose();   
        }
       
        auto time_mult = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_mult);
       
        auto start_t_mult2 = high_resolution_clock::now();
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_M(Hess);
                
        Eigen::VectorXd eigen_hess = eigen_solver_M.eigenvalues().real();
        auto time_mult2 = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_mult2);
        
        if (debug_)
        {
            std::cout << "[HESS] Time matrix: " << (double) time_mult.count() << std::endl; 
            std::cout << "[HESS] Time EIG: " << (double) time_mult2.count() << std::endl;
        }
        
        
        
        return eigen_hess(0);
}

NCertRes NViewCertClass::checkOptimality(const Eigen::VectorXd & sol, 
                                         const Eigen::MatrixXd & C)
{
        /* Main function:
                1. compute multipliers
                2. compute hessian
        */
        
        // 1. Compute multipliers
        Eigen::VectorXd sol_mult; 
       
        auto start_t_mult = high_resolution_clock::now();
        double error_lin = computeMult(sol, C, sol_mult); 
        auto time_mult = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_mult);
       
        // 2. Form and check Hessian
        double cost_sol = sol.dot(sol); 
        
        Eigen::MatrixXd Hess;
        auto start_t_hess = high_resolution_clock::now();
        
        // reduced 
        double min_eigen = computeHessian(cost_sol, sol_mult, Hess); 
        
        auto time_hess = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_hess);
        
      
        if (debug_)
        {
            std::cout << "[OPT] Error linear system for multipliers: " << error_lin << std::endl; 
            std::cout << "[OPT] Minimum eigenvalue Hessian: " << min_eigen << std::endl;        
        }
        
        NCertRes res; 
        res.min_eig = min_eigen; 
        
        res.time_mult = (double) time_mult.count(); 
        res.time_hess = (double) time_hess.count();
        
        if (debug_)
        {
                res.mult = sol_mult; 
                res.Hess = Hess; 
        }
        return res; 

}


void NViewCertClass::printResult(const NCertRes & res)
{
        std::cout << "---------------------\n|       SOLUTION    |\n---------------------\n";
        std::cout << "Minimum eigenvalue Hessian: " << res.min_eig << std::endl; 
        std::cout << "Time for multipliers [nanosecs]: " << res.time_mult << std::endl;
        std::cout << "Time for Hessian [nanosecs]: " << res.time_hess << std::endl;

}
}   // end of namespace NViewtrian
