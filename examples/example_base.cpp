#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>


// include types
#include "NViewsTypes.h"
// include utils
#include "NViewsUtils.h"
// triangulation solver
#include "NViewsClass.h"


// data generation
#include "../utils/generatePointCloud.h"
// ceres solver
#include "../utils/ceresSolver.h"


// for eigendecomposition
#include <eigen3/Eigen/Dense>
#include <Eigen/Eigenvalues> 

#include <chrono>  // timer

using namespace std::chrono;


using namespace std;
using namespace Eigen;
using namespace NViewsTrian;



int main(int argc, char** argv)
{

        std::cout << "Example N view triangulation\n"; 
    
        
        // parameters for estimation
        double noise = 2.0;
        size_t n_points = 1;
        double max_parallax = 1.0;  // in meters
        double focal_length = 512; 
        size_t size_img = 512;
        double max_side = 8.0;
        double dist_center = 3.0;
        double max_rot = 0.5;
        int M_cameras = 100;
          
        std::srand(std::time(nullptr));

                                       
        // generate problem
        PCRes str_out(n_points);
        PCParams str_in; 
        str_in.noise = noise; 
        str_in.focal_length = focal_length; 
        str_in.size_img = size_img; 
        str_in.N_points = n_points; 
        str_in.M_cameras = M_cameras;
        str_in.max_side = max_side; 
        str_in.dist_center = dist_center; 
        
        str_out = generatePointCloud(str_in); //, UtilsTwoView::generateTranslationStereo);    
       
        Matrix3 K = Matrix3::Identity(); 
        K(0,0) = focal_length; 
        K(1,1) = focal_length; 
        K(0,2) = (double) size_img; 
        K(1,2) = (double) size_img;
        Matrix3 iK = K.inverse();

        // generate full graph (M 2) combinations
                
        Eigen::MatrixXd idx_matrix;
        int n_comb = generateM2Comb(M_cameras, idx_matrix);
        
        // generate correspondences
        std::vector<PairObj> set_corr; 
        set_corr.empty();
        for (int jj=0; jj<n_comb; jj++)
        {
                PairObj corr_i; 
                int id1 = idx_matrix(0, jj); 
                int id2 = idx_matrix(1, jj); 
                Matrix3 R1 = str_out.set_rot[id1]; 
                Matrix3 R2 = str_out.set_rot[id2]; 
                Vector3 t1 = str_out.set_trans[id1]; 
                Vector3 t2 = str_out.set_trans[id2]; 
                Matrix4 P1 = Matrix4::Identity(); 
                Matrix4 P2 = Matrix4::Identity(); 
                P1.block<3,3>(0,0) = R1; 
                P1.block<3,1>(0,3) = t1;
                P2.block<3,3>(0,0) = R2; 
                P2.block<3,1>(0,3) = t2;
                
                Matrix4 Prel = P2 * P1.inverse(); 
                Matrix3 Rrel = Prel.block<3,3>(0,0); 
                Vector3 trel = Prel.block<3,1>(0,3);                 
                trel.normalize();
           
                
                
                Matrix3 Ess = Matrix3::Identity(); 
                Matrix3 Tx = Matrix3::Zero(); 
                Tx << 0, -trel(2), trel(1), trel(2), 0, -trel(0), -trel(1), trel(0), 0; 
                // fill T
                Ess = Tx * Rrel;
                Matrix3 F = iK.transpose() * Ess * iK;
                // normalize F
                Eigen::JacobiSVD<Matrix3> svd(F);
                F /= svd.singularValues()(1);
                corr_i.id1 = id1; 
                corr_i.id2 = id2; 
                corr_i.F = Ess; 
                
                corr_i.p1 = iK * str_out.obs[0].col(id1); 
                corr_i.p2 = iK * str_out.obs[0].col(id2);     
                set_corr.push_back(corr_i);        
                
                        
        }
        
        // run correction method
        NViewsClass corr_N_view; 
        // 1. Create constraint matrices
        corr_N_view.createProblemMatrices(set_corr, M_cameras); 
       
       
        // 2. Run correction
        NViewsOptions options_corr; 
        options_corr.save_val_constr = false;
        options_corr.debug = false; 
        NViewsResult res_corr = corr_N_view.correctObservations(options_corr);
        
        // Show results
        corr_N_view.printResult(res_corr); 
        
       
 
        // Matrix projections
        std::vector<Matrix4> proj_s; 
        proj_s.empty();
        std::vector<Vector3> obs_s, obs_init, obs_ref; 
        obs_s.empty(); 
        obs_init.empty(); 
        obs_ref.empty(); 
        
        for (int jc=0; jc<M_cameras;jc++)
        {
                // matrix projection for this camera
                Matrix3 R = str_out.set_rot[jc]; 
                Vector3 t = str_out.set_trans[jc]; 
                Matrix4 P1 = Matrix4::Identity(); 
                P1.block<3,3>(0,0) = R; 
                P1.block<3,1>(0,3) = t;
                proj_s.push_back(P1); 
                
                // observations
                
                Vector3 pt = iK * str_out.obs[0].col(jc); 
                obs_s.push_back(pt);
                
                // update observation init
                Vector3 delta_init; 
                delta_init << res_corr.sol_init( jc*2), res_corr.sol_init(jc*2 + 1), 0;
                obs_init.push_back(pt + delta_init);
                
                // update observation refinenement  
                Vector3 delta_ref; 
                delta_ref << res_corr.sol_final( jc*2), res_corr.sol_final(jc*2 + 1), 0; 
                
                obs_ref.push_back(pt + delta_ref); 
        }
        
        // triangulate point
        Vector3 P_lin; 
        Eigen::VectorXd depths_lin; 
        double error_lin = triangulateNPoint(proj_s, obs_s, P_lin, depths_lin);
       
        Vector3 P_init; 
        Eigen::VectorXd depths_init; 
        double error_init = triangulateNPoint(proj_s, obs_init, P_init, depths_init);
        
        Vector3 P_ref; 
        Eigen::VectorXd depths_ref; 
        double error_ref = triangulateNPoint(proj_s, obs_ref, P_ref, depths_ref);
                                                 
        std::cout << "Number constraints: " << n_comb << std::endl;
        std::cout << "Error linear: " << error_lin << std::endl; 
        std::cout << "error init: " << error_init << std::endl; 
        std::cout << "Error final: " << error_ref << std::endl;

        
        std::cout << "P3d:\n" << str_out.points_3D.col(0) << std::endl;
        std::cout << "P linear:\n" << P_lin << std::endl;
        std::cout << "P init:\n" << P_init << std::endl;
        std::cout << "P ref:\n" << P_ref << std::endl;
        
        
        /** Run ceres **/
        std::vector<std::pair<Eigen::Matrix<double, 3, 4>,Eigen::Vector2d>> vector_pair_ceres_data; 
        vector_pair_ceres_data.reserve(M_cameras); 
         
        for (int jc=0; jc<M_cameras;jc++)
        {
                // matrix projection for this camera
                Matrix3 R = str_out.set_rot[jc]; 
                Vector3 t = str_out.set_trans[jc]; 
                Matrix34 P1; 
                P1.setZero(); 
                P1.block<3,3>(0,0) = R; 
                P1.block<3,1>(0,3) = t;
                                
                // observations
                Vector3 pt = iK * str_out.obs[0].col(jc); 
             
                std::pair<Eigen::Matrix<double, 3, 4>,Eigen::Vector2d> pair_i(P1, pt.topRows(2)); 
                vector_pair_ceres_data.push_back(pair_i);                
        }
  
        auto start_ceres = high_resolution_clock::now();
        Eigen::Vector3d P_ceres = CeresSolver::Triangulate(vector_pair_ceres_data, P_lin); 
        auto time_ceres = duration_cast<nanoseconds>(high_resolution_clock::now() - start_ceres);
        
        std::cout << "Point 3D from ceres:\n" << P_ceres << std::endl; 
        std::cout << "Time ceres: " << (double) time_ceres.count() << std::endl;
        
                             
                                
        return 0;

}  // end of main fcn
