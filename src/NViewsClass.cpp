#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>

// include types
#include "NViewsTypes.h"
// utils
#include "NViewsUtils.h"
// header
#include "NViewsClass.h"
// certifier 
#include "NViewsCertifier.h"

#include <chrono>  // timer

using namespace std::chrono;

namespace NViewsTrian
{
// create problem matrices
void NViewsClass::createProblemMatrices(const std::vector<PairObj> & obj,
                                        const int N_cams)
{
  
        constr_red_.empty(); 
        
        M_ = obj.size();
        N_cams_ = N_cams;
        int s_constr = 2 * N_cams_ + 1;
        for (int i=0; i<M_; i++)
        {       
                Matrix3 F = obj[i].F; 
                Vector3 p1 = obj[i].p1; 
                Vector3 p2 = obj[i].p2;      
                Vector3 Fp2 = F.transpose() * p2; 
                Vector3 Fp1 = F * p1; 
                double e_ep = p2.dot(F * p1);
                int id_1 = (obj[i].id1) * 2; 
                int id_2 = (obj[i].id2) * 2; 
                
               
                
                // create struct
                Constr2View ci; 
                ci.b = e_ep; 
                ci.Fp2 = Fp2.topRows(2); 
                ci.Fp1 = Fp1.topRows(2); 
                ci.F = F.transpose().topLeftCorner(2,2); 
                ci.id_1 = id_1; 
                ci.id_2 = id_2;
                constr_red_.push_back(ci);

        }
        
        constr_are_created_ = true;

}

// correction: init
double NViewsClass::initCorrection(Eigen::VectorXd & sol_init, 
                                   Eigen::MatrixXd & A, 
                                   Eigen::VectorXd & b)
{
        
        A.resize(M_, 2 * N_cams_); 
        b.resize(M_);
        
        A.setZero(); 
        b.setZero(); 
        
        
        for (int i=0; i < M_; i++)
        {                
         
               
                A.block<1,2>(i, constr_red_[i].id_1) = constr_red_[i].Fp2.transpose(); 
                A.block<1,2>(i, constr_red_[i].id_2) = constr_red_[i].Fp1.transpose();
                
                b(i) = constr_red_[i].b;
        }
       
        sol_init.resize(2 * N_cams_);
        double error_sol = solveLinearSystemMinNorm(A.transpose() * A, - A.transpose() * b, sol_init);
        
    
        return error_sol;
        
}




// correction ref
double NViewsClass::refineCorrection(const Eigen::MatrixXd & A0, 
                                     const Eigen::VectorXd & b0, 
                                     const Eigen::VectorXd & sol_init, 
                                     Eigen::VectorXd & sol_ref, 
                                     Eigen::MatrixXd & Ci)
{

        Eigen::MatrixXd C(M_, 2 * N_cams_); 
        Eigen::VectorXd e(M_);
        C = A0; 
        e = b0; 
        
        
        for (int i=0; i < M_; i++)
        {
                
                Vector2 p1 = sol_init.block<2,1>(constr_red_[i].id_1, 0);
                Vector2 p2 = sol_init.block<2,1>(constr_red_[i].id_2, 0);
                
                
                
                C.block<1,2>(i, constr_red_[i].id_1) += p2.transpose() * constr_red_[i].F.transpose(); 
                C.block<1,2>(i, constr_red_[i].id_2) += p1.transpose() * constr_red_[i].F; 
                e(i) -= p1.transpose() * constr_red_[i].F * p2;
}


        Eigen::VectorXd sol_delta(2 * N_cams_);
        double error_sol_t = solveLinearSystemMinNorm(C.transpose() * C, - C.transpose() * e, sol_delta);
        
        
        
        // Save output
        sol_ref.resize(2 * N_cams_);       
        sol_ref = sol_delta; 
        
        Ci.resize(C.rows(), C.cols()); 
        Ci = C; 
        
        return error_sol_t;
        
}   


// correction: K-th refinement
// main function
NViewsResult NViewsClass::correctObservations(NViewsOptions & options)
{       

        if (!constr_are_created_)
        {
                std::cout << "[ERRROR] Constraints are not created!\n";
                return NViewsResult();
        }
        // else        
                
        Eigen::VectorXd sol_i, sol_init;
        Eigen::MatrixXd A0; 
        Eigen::VectorXd b0; 
        
        if (options.debug)
            std::cout << "[CORR] Estimating initial guess\n"; 
            
        auto start_t_init = high_resolution_clock::now();
        double error_init = initCorrection(sol_init, A0, b0);
        auto time_init = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_init);
        
        
        Eigen::VectorXd val_constr_i; 
        double max_constr_val_i = 10.0, sq_constr_i = 10.0;
        auto start_t_ref_i2 = high_resolution_clock::now();
        

        double tot_constr_i = checkConstraints(sol_init, constr_red_, val_constr_i, max_constr_val_i, sq_constr_i);
        
        auto time_ref_i2 = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_ref_i2);
        int k_iter = 0; 
        
        if (options.debug)
        {
            std::cout << "[CORR] Maximum value constraint for initial guess: " << max_constr_val_i << std::endl; 
            std::cout << "[CORR] L-1 norm constraints for initial guess: " << tot_constr_i << std::endl;   
        }
            

        
        
        
        NViewsResult res = NViewsResult(); 
        res.tot_constr_init = tot_constr_i; 
        res.max_constr_init = max_constr_val_i; 
        res.sq_constr_init = sq_constr_i;
        if (options.record_constr)
                {
                        res.rec_constr.empty(); 
                        res.rec_constr.push_back(val_constr_i);
                
                }
        
        if (options.debug)
        {
            std::cout << "[CORR] Refining solution\n--- --- --- --- --- --- --- --- ---\n"; 
        }
        
        sol_i = sol_init;
        double time_ref = 0;
        double diff_sol = 10; 
        double error_lin = 10; 
        Eigen::MatrixXd Cnext = A0; 
        // evaluate stopping condition 
        // We sop if
        // 1. we reach the maximum number of iterations
        // 2. The diff between solutions is less than threshold
        while ( (k_iter < options.max_iters) & (diff_sol > options.max_diff_sol) )
        {
 
                // keep refining
                Eigen::VectorXd sol_next = sol_i;
              
                auto start_t_ref_i = high_resolution_clock::now();
                error_lin = refineCorrection(A0, b0, sol_i, sol_next, Cnext);
                auto time_ref_i = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_ref_i);
                
                diff_sol = (sol_next - sol_i).squaredNorm(); 
                
            
                tot_constr_i = checkConstraints(sol_next, constr_red_, val_constr_i, max_constr_val_i, sq_constr_i);
                
                if (options.debug)
                {
                    std::cout << "[CORR] Iteration #" << k_iter; 
                    std::cout << "            Max constraint: " << max_constr_val_i; 
                    std::cout << "            Total constraint L1: " << tot_constr_i;
                    std::cout << "            Total constraint L2: " << sq_constr_i;
                    std::cout << "            Norm prev solution: " << sol_i.squaredNorm(); 
                    std::cout << "            Norm new solution: " << sol_next.squaredNorm() << std::endl;
                    std::cout << "            Diff between solutions: " << diff_sol << std::endl; 
                }
                
                
                if (options.record_constr)
                {
                        res.rec_constr.push_back(val_constr_i);
                
                }
                time_ref += (double) time_ref_i.count();
                // update iteration
                k_iter++;        
                sol_i = sol_next;
                
        }
        
       
        if (options.debug)
        {
            std::cout << "\n--- --- --- --- --- --- --- --- ---\n";
            std::cout << "[CORR] Nr. iterations: " << k_iter; 
            std::cout <<      "       Max constraint: " << max_constr_val_i; 
            std::cout << "            Total constraint L1: " << tot_constr_i;
            std::cout << "            Total constraint L2: " << sq_constr_i;
            std::cout << "            Norm intial solution: " << sol_init.squaredNorm(); 
            std::cout <<   "          Norm final solution: " << sol_i.squaredNorm() << std::endl;
            std::cout << "            Diff between solutions: " << diff_sol << std::endl; 
        }
        
        // Check optimality with sufficient condition

        
         // suff. condition       
        NViewCertClass cert_obj(N_cams_, constr_red_, options.debug_cert); 
    
        // call function
        auto start_t_cert = high_resolution_clock::now();
        NCertRes res_cert = cert_obj.checkOptimality(sol_i, Cnext); 
        auto time_cert = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_cert);
                
        
        
        /* Save results */
        
        res.min_eig = res_cert.min_eig; 
        res.time_cert_mult = res_cert.time_mult;
        res.time_cert_hess = res_cert.time_hess;
                
                
        res.n_iters = k_iter;
        res.sol_init = sol_init;
        res.sol_final = sol_i;
        res.max_constr = max_constr_val_i; 
        res.tot_constr = tot_constr_i; 
        res.sq_constr = sq_constr_i; 
        res.diff_sol = diff_sol; 
        if (options.save_val_constr)
                res.all_constr = val_constr_i;
        res.time_init = (double) time_init.count();
        res.time_ref = time_ref;
        res.time_opt = (double) time_cert.count(); 
        res.error_lin = error_lin;
        return res;
}


// Print solution
void NViewsClass::printResult(NViewsResult & res_corr)
{

        std::cout << "---------------------\n|       SOLUTION    |\n---------------------\n"; 
        std::cout << "Number iterations: " << res_corr.n_iters << std::endl;
        // std::cout << "Initial solution:\n" << res_corr.sol_init << std::endl; 
        // std::cout << "Final solution:\n" << res_corr.sol_final << std::endl; 
        std::cout << "Maximum value of constraint: "  << res_corr.max_constr << std::endl; 
        std::cout << "Norm of the constraints [L1]: " << res_corr.tot_constr << std::endl; 
        std::cout << "Norm of the constraints L2: "   <<  res_corr.sq_constr << std::endl;
        std::cout << "Difference between the last two solutions: " << res_corr.diff_sol << std::endl;
        std::cout << "Error linear system: " << res_corr.error_lin << std::endl; 
        std::cout << "Minimum eigenvalue Hessian: " << res_corr.min_eig << std::endl; 
        if (res_corr.all_constr.size() > 0)
                std::cout << "value of all the constraints:\n" << res_corr.all_constr << std::endl;
        if (res_corr.rec_constr.size() > 0)
                {
                        std::cout << "The value of the constraints for the iterations are:\n"; 
                        for (int j = 0; j < res_corr.rec_constr.size(); j++)
                        {
                                std::cout << "Iteration #" << j << std::endl; 
                                std::cout << res_corr.rec_constr[j].transpose() << std::endl;
                                
                        }
                
                }
         std::cout << "Time init [nanosecs]: " << res_corr.time_init << std::endl; 
         std::cout << "Time ref [nanosecs]: " << res_corr.time_ref << std::endl;
         std::cout << "Time cert [nanosecs]: " << res_corr.time_opt << std::endl;
}







}  // end of namespace
