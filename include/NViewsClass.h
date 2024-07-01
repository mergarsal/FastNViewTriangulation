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
               
        
        /* Struct with options */
        struct NViewsOptions
        {
                int max_iters = 5; 
                double max_constr = 1e-10; 
                double max_tot_constr = 1e-09;   
                double max_diff_sol = 3e-10;     
        
                bool save_val_constr = false; // record the final values
                bool record_constr = false;  // record all the values
                
                bool debug = true;  // debug flag
                bool debug_cert = false;  // debug flag for certifier
                NViewsOptions(){};
        };  // end of struct options
        
        
        /* Struct for the result */
        struct NViewsResult
        {
                int n_iters = 0; 
                Eigen::VectorXd sol_init; 
                Eigen::VectorXd sol_final; 
                double max_constr = 10; 
                double tot_constr = 10; 
                double sq_constr = 10;
                
                double error_lin = 10; 
                
                double diff_sol = 10;
                Eigen::VectorXd all_constr;
                std::vector<Eigen::VectorXd> rec_constr = {};
                
                double tot_constr_init = 10; 
                double max_constr_init = 10;
                double sq_constr_init = 10; 
                
                double time_init = 100; 
                double time_ref = 100;
                double time_opt = 100;  
                double time_cert_mult = 100; 
                double time_cert_hess = 100; 
                
                double min_eig = -1; 
                double error_mult = -1; 
                
                
        
                NViewsResult(){};
        };  // end of struct result
        
        
        class NViewsClass
        {
                public:
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
                        
                        NViewsClass(){};
                        ~NViewsClass(){};
                        
                        /* Create problem matrices */
                        void createProblemMatrices(const std::vector<PairObj> & obj,
                                                   const int N_cams);
                                        
                                        
                        /* Initialize the solution sol_init */
                        double initCorrection(Eigen::VectorXd & sol_init, Eigen::MatrixXd & A, Eigen::VectorXd & b);
                        
                        /* Single refinement of the solution sol_init */
                        double refineCorrection(const Eigen::MatrixXd & A, 
                                                const Eigen::VectorXd & b, 
                                                const Eigen::VectorXd & sol_init, 
                                                Eigen::VectorXd & sol_ref, 
                                                Eigen::MatrixXd & Ci);
                        /* Main function */    
                        NViewsResult correctObservations(NViewsOptions & options);
                        
                        /* Show results */
                        void printResult(NViewsResult & res_corr);
                                                
                        void getConstrRed(std::vector<Constr2View> &C){ C = constr_red_;};
                        
                        int checkOpt(const Eigen::VectorXd & sol_init, double & time_opt, int &, int & ); 
                        
                private:
                        int M_;  // number constraints
                        int N_cams_;   // number cameras
                        std::vector<Constr2View> constr_red_;      // reduced constraints
                        bool constr_are_created_;  // true if the constraints are done
                
        };  // end of main class



}  // end of namespace
