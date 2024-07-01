#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <chrono>  // timer
#include <sstream>
#include <fstream>  // for the file

// include helper 
#include "experimentsHelper.h"

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

// for eigen decomposition
#include <eigen3/Eigen/Dense>
#include <Eigen/Eigenvalues> 

#include <chrono>  // timer

using namespace NViewsTrian;
using namespace std::chrono;

#define SAVEPROB TRUE

enum class methodGenPose{
        ORBITAL = 0, 
        LATERAL,
        GENERAL 
};  // end of enum class

Eigen::Vector3d returnThisTranslation( double max_parallax, const Eigen::Vector3d & dir_parallax)
{
        return (dir_parallax);

}; 

int main(int argc, char** argv)
{
          std::cout << "Generic test:\nN views triangulation comparison\n"; 
          
          
   
           /* Read params from input */ 
   
           string name_in_params = "basic_params.txt"; 
           
           /* Read the name of the file */
           if (argc > 1)
                name_in_params = argv[1]; 
                
          
          SceneOptions options; 
           
          std::cout << "Generic test file !\n";
          std::cout << "Input for the test: " << name_in_params << std::endl; 
          
          
          // read params from file
          bool valid_options = readOptionsFromFile(name_in_params, options);   
          

          std::srand(std::time(nullptr));
          
          int n_points = options.n_points; 
          
          int size_img_i = 512;
          
    for (size_t noise_id=0; noise_id < options.n_noise; noise_id++)
    {
        double noise_i = options.noise[noise_id];

        for (size_t cam_id = 0; cam_id < options.n_arr_cams; cam_id++)
        {
                int M_cam_i = options.arr_cams[cam_id];
        
        
                for (size_t par_id = 0; par_id < options.n_parallax; par_id++)
                {
                        double par_i = options.max_parallax[par_id];    
        
        
                        for (size_t focal_id = 0; focal_id < options.n_focal; focal_id++)
                        {
                                double focal_i = options.focal_length[focal_id];
                                
                               for (size_t d_c_id = 0; d_c_id < options.n_dist_centers; d_c_id++)
                               {
                                        double dist_center_i = options.dist_centers[d_c_id]; 
                                         
                                         // This file saves all our resutls
                                         auto name_f_sol = "res/sol_noise_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream fsol(name_f_sol);
                                         
                                         auto name_f_lin_3d = "res/lin_3D_noise_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream flin3d(name_f_lin_3d);
                                         
                                         auto name_f_3d = "res/err_3D_noise_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream f3d(name_f_3d);
                                         
                                         auto name_f_l2 = "res/err_l2_noise_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream fl2(name_f_l2);
         
                                         
                                         
                                         auto name_f_l1 = "res/err_l1_noise_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream fl1(name_f_l1);
                                         
                                         auto name_f_linfty = "res/err_linfty_noise_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream flinfty(name_f_linfty);
                                         
                                         auto name_f_times = "res/times_noise_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream ftime(name_f_times);
                                         
                                         auto name_f_exp = "res/times_exp_noise_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream ft_exp(name_f_exp);
                                         
                                         auto name_diff = "res/diff_noise_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream fdiff(name_diff);
                                         
                                      
                                         #if SAVEPROB
                                         {
                                                 auto name_prob = "res/prob_" + std::to_string(noise_i) 
                                                                             + "_cams_" + std::to_string((int)M_cam_i) 
                                                                             + "_par_" + std::to_string(par_i) 
                                                                             + "_focal_" + std::to_string((int)focal_i) 
                                                                             + "_center_" + std::to_string(dist_center_i) 
                                                                             + ".txt";
                                                 std::ofstream fprob(name_prob);
                                         }
                                         #endif
                                         auto name_constr = "res/constr_" + std::to_string(noise_i) 
                                                                     + "_cams_" + std::to_string((int)M_cam_i) 
                                                                     + "_par_" + std::to_string(par_i) 
                                                                     + "_focal_" + std::to_string((int)focal_i) 
                                                                     + "_center_" + std::to_string(dist_center_i) 
                                                                     + ".txt";
                                         std::ofstream fconstr(name_constr);
                                         
                                                                                
                                         
                                       for (size_t n_iter = 0; n_iter < options.max_iter; n_iter++)
                                       {
                                         
                                               // define struct with params
                                               PCParams str_in = PCParams(); 
                                               
                                               str_in.focal_length = focal_i; 
                                               str_in.N_points = n_points; 
                                               str_in.noise = noise_i; 
                                               str_in.size_img = size_img_i; 
                                               str_in.max_side = 8.0; 
                                               str_in.std_pc = (double) dist_center_i / 4.0; 
                                               str_in.dist_center = dist_center_i; 
                                               str_in.max_parallax = par_i;
                                               str_in.max_angle = options.max_rotation; 
                                               str_in.M_cameras = M_cam_i;

                                               // Select pose generation 
                                               // param: options.method_trans in {1, 2, 3}
                                               methodGenPose m_gen_pose = static_cast<methodGenPose>(options.method_trans);

                                               
                                               // generate problem
                                               PCRes str_out = PCRes(); 
                                               std::cout << "Selection method for pose generation\n"; 
                                               
                                               switch (m_gen_pose)
                                               {
                                                case methodGenPose::ORBITAL:
                                                {
                                                        std::cout << "[ORBITAL CAMERA]\n";
                                                        str_in.max_parallax = 5;  // radius circle
                                                        
                                                        Eigen::Vector3d dist_vector; 
                                                        dist_vector << dist_center_i, 0, 0; 
                                                        str_in.dir_parallax = dist_vector;
                                                        
                                                        str_in.max_angle = dist_center_i; 
                                                        // generateOrbitalRotation takes the distance to center and the translation vector
                                                        
                                                        // In this configuration, rotation and translation depends on dist_center_i 
                                                        // and radius = max_parallax = 5 units
                                                        str_in.noise_rot = options.noise_rot; 
                                                        str_in.noise_trans = options.noise_trans;
                                                        
                                                        str_out = generatePointCloud(str_in, generateOrbitalTranslation, generateOrbitalRotation); 
                                                }
                                                break;
                                                
                                                case methodGenPose::LATERAL: 
                                                        std::cout << "[LATERAL CAMERA]\n";
                                                        // In this configuration, 
                                                        // translation has form X, 0 0
                                                        // and rotation is identity
                                                        
                                                        str_in.max_angle = 0.0;                 
                                                        str_in.max_parallax = par_i;                                       
                                                        str_in.noise_trans = options.noise_trans; 
                                                        str_in.noise_rot = options.noise_rot; 
                                                        str_out = generatePointCloud(str_in, generateTranslationStereo);                                                 
                                                break;
                                                
                                                /*
                                                case methodGenPose::GENERAL: 
                                                        std::cout << "[GENERAL CAMERA]\n";
                                                        str_out = generatePointCloud(str_in); 
                                                break;
                                                */
        
                                                default:
                                                        std::cout << "[GENERAL CAMERA]\n";
                                                        str_in.max_angle = options.max_rotation;                 
                                                        str_in.max_parallax = par_i;                                       
                                                        str_in.noise_trans = options.noise_trans; 
                                                        str_in.noise_rot = options.noise_rot; 
                                                        str_out = generatePointCloud(str_in);                                      
                                                break;
                                               
                                               }
                                               
                                               Matrix3 K = Matrix3::Identity(); 
                                               K(0,0) = focal_i; 
                                               K(1,1) = focal_i; 
                                               K(0,2) = (double) size_img_i; 
                                               K(1,2) = (double) size_img_i;
                                               Matrix3 iK = K.inverse();

                                                
                                                // extract data
                                                // generate full graph (M 2) combinations
                
                                                Eigen::MatrixXd idx_matrix;
                                                int n_comb = generateM2Comb(M_cam_i, idx_matrix);
                                                
                                                                                                        
                                                // generate correspondences
                                                std::vector<PairObj> set_corr; 
                                                set_corr.empty();
                                                for (int jj=0; jj<n_comb; jj++)
                                                {
                                                        PairObj corr_i; 
                                                        int id1 = idx_matrix(0, jj); 
                                                        int id2 = idx_matrix(1, jj); 
                                                        Eigen::Matrix3d R1 = str_out.set_rot[id1]; 
                                                        Eigen::Matrix3d R2 = str_out.set_rot[id2]; 
                                                        Eigen::Vector3d t1 = str_out.set_trans[id1]; 
                                                        Eigen::Vector3d t2 = str_out.set_trans[id2]; 
                                                        Eigen::Matrix4d P1 = Eigen::Matrix4d::Identity(); 
                                                        Eigen::Matrix4d P2 = Eigen::Matrix4d::Identity(); 
                                                        P1.block<3,3>(0,0) = R1; 
                                                        P1.block<3,1>(0,3) = t1;
                                                        P2.block<3,3>(0,0) = R2; 
                                                        P2.block<3,1>(0,3) = t2;
                                                        
                                                        Eigen::Matrix4d Prel = P2 * P1.inverse(); 
                                                        Eigen::Matrix3d Rrel = Prel.block<3,3>(0,0); 
                                                        Eigen::Vector3d trel = Prel.block<3,1>(0,3);                 
                                                        trel.normalize();
                                                   
                                                        
                                                        
                                                        Eigen::Matrix3d Ess = Eigen::Matrix3d::Identity(); 
                                                        Eigen::Matrix3d Tx = Eigen::Matrix3d::Zero(); 
                                                        Tx << 0, -trel(2), trel(1), trel(2), 0, -trel(0), -trel(1), trel(0), 0; 
                                                        // fill T
                                                        Ess = Tx * Rrel;
                                                        corr_i.id1 = id1; 
                                                        corr_i.id2 = id2; 
                                                        corr_i.F = Ess;   
                                                        set_corr.push_back(corr_i);   
                                                }
                                                
                                                // Save pose 
                                                #if SAVEPROB
                                                {
                                                        for (int jj=0; jj < M_cam_i; jj++)
                                                        {
                                                                Eigen::Matrix3d R = str_out.set_rot[jj]; 
                                                                Eigen::Vector3d t = str_out.set_trans[jj]; 
                                                                fprob << t(0) << "," << t(1) << "," << t(2) << ",";
                                                                
                                                                for (int id_r = 0; id_r < 3; id_r++)
                                                                {
                                                                        for (int id_c=0; id_c < 3; id_c++)
                                                                                fprob << R(id_r, id_c) << ",";
                                                                } 
                                                                fprob << std::endl;
                                                        }
                                                }
                                                #endif
                                              
                                               // for each point
                                               int idx = 0; 
                                               for (idx = 0; idx < n_points; idx++)
                                               {
                                                
                                                        // 1. Loop through cameras
                                                        for (int jj=0; jj<n_comb; jj++)
                                                        {
                                                                int id1 = idx_matrix(0, jj); 
                                                                int id2 = idx_matrix(1, jj); 
                                                                set_corr[jj].p1 = iK * str_out.obs[idx].col(id1); 
                                                                set_corr[jj].p2 = iK * str_out.obs[idx].col(id2);   
                                                             
                                                        }
                                                        
                                                        Eigen::Vector3d X = str_out.points_3D.col(idx);
                                                        
                                                        // run correction method
                                                        NViewsClass corr_N_view; 
                                                        // 1. Create constraint matrices
                                                        corr_N_view.createProblemMatrices(set_corr, M_cam_i); 
                                                       
                                                        // 2. Run correction
                                                        NViewsOptions options_corr;                                                         
                                                        options_corr.save_val_constr = false;
                                                        options_corr.debug = false; 
                                                        
                                                        auto start_t_ours = high_resolution_clock::now();
                                                        NViewsResult res_corr = corr_N_view.correctObservations(options_corr);
                                                        auto time_ours = duration_cast<nanoseconds>(high_resolution_clock::now() - start_t_ours);
                                                        
                                                          
                                        
                                                        // c. Reconstruct 3D point by linear method
                                                    
                                                        // Matrix projections
                                                        std::vector<Eigen::Matrix4d> proj_s; 
                                                        proj_s.empty();
                                                        std::vector<Eigen::Vector3d> obs_s, obs_init, obs_ref; 
                                                        obs_s.empty(); 
                                                        obs_init.empty(); 
                                                        obs_ref.empty(); 
                                                        
                                                        for (int jc=0; jc<M_cam_i;jc++)
                                                        {
                                                                // matrix projection for this camera
                                                                Eigen::Matrix3d R = str_out.set_rot[jc]; 
                                                                Eigen::Vector3d t = str_out.set_trans[jc]; 
                                                                Eigen::Matrix4d P1 = Eigen::Matrix4d::Identity(); 
                                                                P1.block<3,3>(0,0) = R; 
                                                                P1.block<3,1>(0,3) = t;
                                                                proj_s.push_back(P1); 
                                                                
                                                                // observations
                                                                
                                                                Eigen::Vector3d pt = iK * str_out.obs[idx].col(jc); 
                                                                obs_s.push_back(pt);
                                                                
                                                                // update observation init
                                                                Eigen::Vector3d delta_init; 
                                                                delta_init << res_corr.sol_init( jc*2), res_corr.sol_init(jc*2 + 1), 0;
                                                                obs_init.push_back(pt + delta_init);
                                                                
                                                                // update observation refinenement  
                                                                Eigen::Vector3d delta_ref; 
                                                                delta_ref << res_corr.sol_final( jc*2), res_corr.sol_final(jc*2 + 1), 0; 
                                                                obs_ref.push_back(pt + delta_ref);
                                                        }
                                                        
                                                                                                          
                                                        // triangulate point
                                                        Eigen::Vector3d P_lin; 
                                                        Eigen::VectorXd depths_lin; 
                                                        double error_lin = triangulateNPoint(proj_s, obs_s, P_lin, depths_lin);
                                                        
                                                        Eigen::Vector3d P_init; 
                                                        Eigen::VectorXd depths_init; 
                                                        double error_init = triangulateNPoint(proj_s, obs_init, P_init, depths_init);
                                                        
                                                        Eigen::Vector3d P_ref; 
                                                        Eigen::VectorXd depths_ref; 
                                                        double error_ref = triangulateNPoint(proj_s, obs_ref, P_ref, depths_ref);
                                                        
                                                                                                                                                                
                                                        /** Run ceres **/
                                                        std::vector<std::pair<Eigen::Matrix<double, 3, 4>, Eigen::Vector2d>> vector_pair_ceres_data; 
                                                        vector_pair_ceres_data.reserve(M_cam_i); 
                                                         
                                                        for (int jc=0; jc<M_cam_i;jc++)
                                                        {
                                                                // matrix projection for this camera
                                                                Eigen::Matrix3d R = str_out.set_rot[jc]; 
                                                                Eigen::Vector3d t = str_out.set_trans[jc]; 
                                                                Eigen::Matrix<double, 3, 4> P1; 
                                                                P1.setZero(); 
                                                                P1.block<3,3>(0,0) = R; 
                                                                P1.block<3,1>(0,3) = t;
                                                                                
                                                                // observations
                                                                Eigen::Vector3d pt = obs_ref[jc]; 
                                                             
                                                                std::pair<Eigen::Matrix<double, 3, 4>,Eigen::Vector2d> pair_i(P1, pt.topRows(2)); 
                                                                vector_pair_ceres_data.push_back(pair_i);                
                                                        }
                                                  
                                                        /** Run ceres **/
                                                        Eigen::Vector3d P_ceres = CeresSolver::Triangulate(vector_pair_ceres_data, P_ref); 
                                                        
                                                        Eigen::VectorXd sol_ceres = res_corr.sol_init; 
                                                        for (int jc=0; jc<M_cam_i;jc++)
                                                        {
                                                            Eigen::Matrix3d R = str_out.set_rot[jc]; 
                                                            Eigen::Vector3d t = str_out.set_trans[jc]; 
                                                            Eigen::Matrix<double, 3, 4> P1; 
                                                            P1.setZero(); 
                                                            P1.block<3,3>(0,0) = R; 
                                                            P1.block<3,1>(0,3) = t;
                                                                
                                                            Eigen::Vector4d P_ceres_h; 
                                                            P_ceres_h << P_ceres, 1; 
                                                            
                                                            Eigen::Vector3d obs_ceres_i = P1 * P_ceres_h;
                                                            obs_ceres_i /= obs_ceres_i(2);
                                                            sol_ceres.block<2,1>(jc * 2,0) = (obs_ceres_i - iK * str_out.obs[idx].col(jc)).topRows(2);
                                                        
                                                        }
                                                        
                                                        /* Save results */ 
                                                        // solution of problem 
                                                        fsol << P_lin(0) << "," << P_lin(1) << "," << P_lin(2) << "," ; 
                                                        fsol << P_init(0) << "," << P_init(1) << "," << P_init(2) << "," ; 
                                                        fsol << P_ref(0) << "," << P_ref(1) << "," << P_ref(2) << "," ; 
                                                        fsol << P_ceres(0) << "," << P_ceres(1) << "," << P_ceres(2) << "," ; 
                                                        fsol << std::endl; 
                                                        
                                                        // Euclidean distance 3D point
                                                        f3d << (P_ceres-P_lin).norm() << ",";                 
                                                        f3d << (P_ceres-P_init).norm() << ",";       
                                                        f3d << (P_ceres-P_ref).norm(); 
                                                        f3d << std::endl;
                                                        
                                                        // Error for the linear method
                                                        flin3d << error_lin << ","; 
                                                        flin3d << error_init << ",";
                                                        flin3d << error_ref << ","; 
                                                        flin3d << std::endl;                                                         
                                                        
                                                        // l2 norm for observations 
                                                        fl2 << res_corr.sol_init.squaredNorm()   << ","; 
                                                        fl2 << res_corr.sol_final.squaredNorm()   << ","; 
                                                        fl2 << sol_ceres.squaredNorm()   << ","; 
                                                        fl2 << std::endl;
                                                        
                                                        // L1 norm for observations 
                                                        fl1 << res_corr.sol_init.lpNorm<1>() << ","; 
                                                        fl1 << res_corr.sol_final.lpNorm<1>()<< ","; 
                                                        fl1 << sol_ceres.lpNorm<1>()<< ","; 
                                                        fl1 << std::endl;
                                                        
                                                        // Linfty norm for observations 
                                                        flinfty << res_corr.sol_init.lpNorm<Eigen::Infinity>() << ","; 
                                                        flinfty << res_corr.sol_final.lpNorm<Eigen::Infinity>() << ","; 
                                                        flinfty << sol_ceres.lpNorm<Eigen::Infinity>() << ","; 
                                                        flinfty  << std::endl;
                                                        
                                                        
                                                        ftime << res_corr.time_init << ",";
                                                        ftime << (double) time_ours.count() << ",";  
                                                        ftime << M_cam_i << ","; 
                                                        ftime << std::endl;        
                                                        
                                                        ft_exp <<  res_corr.time_init << ","; 
                                                        ft_exp <<  res_corr.time_ref << ","; 
                                                        ft_exp <<  res_corr.n_iters << ","; 
                                                        ft_exp <<  res_corr.time_opt << ","; 
                                                        ft_exp <<  res_corr.time_cert_mult << ","; 
                                                        ft_exp <<  res_corr.time_cert_hess << ","; 
                                                        ft_exp <<  res_corr.min_eig << ","; 
                                                        ft_exp <<  std::endl;        
                                                            
                                                        
                                                        /* Save diff wrt HS */
                                                        fdiff << (res_corr.sol_init - sol_ceres).norm()<< ","; 
                                                        fdiff << (res_corr.sol_final - sol_ceres).norm()<< ",";
                                                        fdiff << std::endl;
                                                                                                                
                                                                                                                                                           
                                                        /* epipolar constraint */                                 
                                                        
                                                        // compute and save epipolar error 
                                                        fconstr << res_corr.max_constr_init << ","; 
                                                        fconstr << res_corr.max_constr << ","; 
                                                        fconstr << res_corr.tot_constr_init << ","; 
                                                        fconstr << res_corr.tot_constr << ",";                                                         
                                                        fconstr << res_corr.sq_constr_init << ",";
                                                        fconstr << res_corr.sq_constr << ",";
                                                        fconstr << res_corr.error_lin << ",";
                                                        fconstr << std::endl;  
                                                        
                                                        
                                                        
                                                }  // end of each point             
                                                } // end of each iteration 
                                                fsol.close();    // solution
                                                flin3d.close();  // error linear method
                                                f3d.close();     // error 3d point
                                                fl2.close();     // error l2 obs
                                                fl1.close();     // error l1 obs
                                                flinfty.close(); // error linfty obs
                                                ftime.close();   // time 
                                                ft_exp.close();   // time dual diagonal
                                                fdiff.close();   // diff wrt HS
                                                #if SAVEPROB
                                                        fprob.close();  // close the problem file if opened
                                                #endif
                                                fconstr.close(); // epipolar constraint
                                                
                                }  // end for dist center
      
                         }  // enf for focal
       
                 }  // end for parallax
      
        }  // end for M_cameras
      
      }  // end for noise              
       
  return 0;

}  // end of main fcn
