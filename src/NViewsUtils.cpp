#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>

// include types
#include "NViewsTypes.h"

// include header
#include "NViewsUtils.h"

#define EIGEN_USE_LAPACKE_STRICT
// for eigendecomposition
#include <eigen3/Eigen/Dense>
#include <Eigen/Eigenvalues> 

#include <chrono>  // timer



using namespace std::chrono;
using namespace std;


namespace NViewsTrian
{

// Check constraints
double checkConstraints(const Eigen::VectorXd & sol, 
                        std::vector<Eigen::MatrixXd> & constr, 
                        Eigen::VectorXd & val_constr,
                        double & max_constr_val)
{
        // Evaluate the constraints at the given solution
        int M = constr.size(); 
        val_constr.resize(M); 
        val_constr.setZero(); 
        
        double tot_constr = 0.0; 
        max_constr_val = -10.0;
        Eigen::VectorXd w(sol.size() + 1); 
        w << sol , 1; 
        for (int i=0; i < M; i++)
        {
                double val_i = w.dot(constr[i] * w);
                // std::cout << "Val constraint: " << val_i << std::endl; 
                tot_constr += val_i;
                val_constr(i) = val_i;
                if (val_i > max_constr_val)
                        max_constr_val = val_i;
        }
        // std::cout << "Value constraints:\n" << val_constr << std::endl;
        return tot_constr;
}

// Check constraints with reduced constraints
double checkConstraints(const Eigen::VectorXd & sol, 
                        std::vector<Constr2View> & constr, 
                        Eigen::VectorXd & val_constr,
                        double & max_constr_val, 
                        double & sq_constr_val)
{
        // Evaluate the constraints at the given solution
        int M = constr.size(); 
        val_constr.resize(M); 
        val_constr.setZero(); 
        
        double tot_constr = 0.0; 
        max_constr_val = -10.0;
        sq_constr_val = 0.0;
        for (int i=0; i < M; i++)
        {
               
                Vector2 p1 = sol.block<2,1>(constr[i].id_1, 0);
                Vector2 p2 = sol.block<2,1>(constr[i].id_2, 0);
                double val_i = constr[i].b + p1.transpose() * constr[i].F * p2 + p1.dot(constr[i].Fp2) + p2.dot(constr[i].Fp1); 
                
                // std::cout << "Val constraint: " << val_i << std::endl; 
                
                val_constr(i) = val_i;
                double abs_val_i = (val_i > 0) ? val_i : -val_i;
                // std::cout << "Val i: " << val_i << std::endl; 
                // std::cout << "Abs val i: " << abs_val_i << std::endl;
                tot_constr += abs_val_i;
                sq_constr_val += abs_val_i * abs_val_i;
                // std::cout << "Tot constr: " << tot_constr << std::endl;
                if (abs_val_i > max_constr_val)
                        max_constr_val = abs_val_i;
        }
        // std::cout << "Squared norm of the constraints: " << sq_constr_val << std::endl;
        // std::cout << "Value constraints:\n" << val_constr << std::endl;
        return tot_constr;
}
       

// solve linear system minimum norm
double solveLinearSystemMinNorm(const Eigen::MatrixXd & A, 
                                const Eigen::VectorXd & b,
                                Eigen::VectorXd & sol)
{
        // tol_rank = 1e-03 * 1e-03;
        double tol_rank = 1e-03 * 1e-03 * 1e-03;  // 1e-03 works fine with simple approach
        
        Eigen::VectorXd y_sol;
        // std::cout << "Calling min norm\n"; 
        auto start_t_init = high_resolution_clock::now();
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(A.rows(),
                                                A.cols());
        cod.setThreshold(tol_rank);
        cod.compute(A);
        auto time_init = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_init);
        
     
        
        auto start_t_init2 = high_resolution_clock::now();
        y_sol = cod.solve(b);
        auto time_init2 = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_init2);
        
        
        // std::cout << "Rank A: " << cod.rank() << std::endl;
        // std::cout << "[MIN] Compute COD: " << (double) time_init.count() << std::endl; 
        // std::cout << "[MIN] Solve system: " << (double) time_init2.count() << std::endl; 
        
        Eigen::VectorXd e_lin = A * y_sol - b;
        // std::cout << "error LS: " << e_lin.squaredNorm() << std::endl;
        double error_sol = e_lin.squaredNorm();
        sol.resize(A.cols(), 1); 
        sol = y_sol;
        return error_sol;
}        





double triangulateNPoint(const std::vector<Eigen::Matrix<double, 3, 4>> & proj_s,
                         const std::vector<Eigen::Vector3d> & obs_s, 
                         Eigen::Vector3d & P_3d, 
                         Eigen::VectorXd & depths)
{

        std::vector<Matrix4> P4 = {};
        int N_cams = proj_s.size(); 
        P4.reserve(N_cams); 
        
        for (int i=0; i < N_cams; i++)
        {
                Eigen::Matrix4d Pi = Eigen::Matrix4d::Identity();
                
                Pi.block<3,4>(0, 0) = proj_s[i];
                P4.push_back(Pi);
        }
        
        return (triangulateNPoint(P4, obs_s, P_3d, depths));

}             
// triangulate point
double triangulateNPoint(const std::vector<Matrix4> & proj_s,
                         const std::vector<Eigen::Vector3d> & obs_s, 
                         Eigen::Vector3d & P_3d, 
                         Eigen::VectorXd & depths)
{

        int N_cams = proj_s.size();
        Eigen::MatrixXd P(N_cams * 3, N_cams + 4);
        P.setZero();
        for (int i=0; i < N_cams; i++)
        {
                const int ri = (i)*3;
                // int rf = (i-1)*3 + 2; 
                P.block<3,4>(ri, 0) = proj_s[i].block<3,4>(0,0);
                P.block<3,1>(ri, 4+i) = obs_s[i]; 
        }
        
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(P, Eigen::ComputeFullV);
        // std::cout << "Singular values P:\n" << svd.singularValues() << std::endl; 
        P_3d.setZero(); 
        Eigen::Vector4d P_hom = svd.matrixV().topRightCorner(4, 1);
        P_3d = P_hom.topRows(3) / P_hom(3);
                
        depths.resize(N_cams); 
        depths.setZero();
        depths = svd.matrixV().bottomRightCorner(N_cams, 1);
        
        int last_id = std::min(N_cams * 3, N_cams + 4) - 1; 
        
        double error_lin = svd.singularValues()(last_id); 

        return error_lin;
}



}   // end of namespace NViewtrian

