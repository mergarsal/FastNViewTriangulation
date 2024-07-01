#pragma once 

#include <Eigen/Core>
#include <functional>
#include <vector>

namespace NViewsTrian
{

typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 3, 3> Matrix3;
typedef Eigen::Matrix<double, 4, 4> Matrix4;

/* Generate random translation */
using GenerateTranslation = std::function<Vector3(const double max_parallax, const Vector3& direction_parallax)>;
/* Generate random rotation */
using GenerateRotation = std::function<Matrix3(const double max_angle, const Vector3& dir_rot, const Vector3& dir_trans)>;
/* Generate random perturbation for translation */
using PerturbTranslation = std::function<Vector3(const double max_parallax, const Vector3& direction_parallax)>;
/* Generate random perturbation for rotation */
using PerturbRotation = std::function<Matrix3(const double max_angle, const Matrix3& rot)>;                
                               
// generate random translation with the specified norm
Vector3 generateRandomTranslationDefault( double max_parallax, const Vector3 & dir_parallax);
// generate random rotation with maxAngle
Matrix3 generateRandomRotationDefault( double maxAngle, const Vector3 & dir_rot, const Vector3& dir_trans); 


// generate random perturbation for translation 
Vector3 perturbRandomTranslationDefault( double noise, const Vector3 & trans);
// generate random perturbation for translation 
Matrix3 perturbRandomRotationDefault( double noise, const Matrix3 & rot); 

int generateM2Comb(const int M,
                      Eigen::MatrixXd & comb_idx);            

/* Options for the point cloud generation */
struct PCParams
{
                   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                   
                   double focal_length = 512;           // in pixels
                   size_t size_img = 1024;
                   size_t N_points = 1;
                   double noise = 0.1;                  // in pixels
                   int max_iter = 500; 
                   double max_side = 8;
                   double std_pc = 4.0; 
                   double dist_center = 8.0; 
                   int M_cameras = 2;
                   
                   // params for relpose
                   double max_parallax = 2.0;               // in meters
                   double max_angle = 0.5;           // in degrees
                   
                   double noise_trans = 0.0; 
                   double noise_rot = 0.0; 
                   
                   Vector3 dir_parallax; 
                   Vector3 dir_rotation;
                   
                   // constructor
                   PCParams(){}; 



};  // end of struct PCParams


/* Result of the point cloud generation */
struct PCRes
{
                 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                   
                    // poses 
                    std::vector<Matrix3> set_rot; 
                    std::vector<Vector3> set_trans;
                    
                    // Observations
                    std::vector<Eigen::MatrixXd>  obs;  // noisy observations
                    // 3D world points
                    Eigen::MatrixXd  points_3D;
                    
                    // constructor
                    /// default
                    PCRes()
                    {
                    points_3D = Eigen::MatrixXd(3, 1); 
                    obs = {};  
                    set_rot = {}; 
                    set_trans = {};                   
                    }; 
                     
                    PCRes(size_t n_points) {
                    points_3D = Eigen::MatrixXd(3, n_points); 
                    obs = {};  
                    set_rot = {}; 
                    set_trans = {};       
                    }; 
        
}; // end of PCRes  
  
  
PCRes generatePointCloud(PCParams & options, 
                         const GenerateTranslation& generateTranslation = generateRandomTranslationDefault,
                         const GenerateRotation& generateRotation = generateRandomRotationDefault,
                         const PerturbTranslation& perturbTranslation = perturbRandomTranslationDefault,
                         const PerturbRotation& perturbRotation = perturbRandomRotationDefault);
                         
 




/** Some special functions  **/
Vector3 generateTranslationForward( double max_parallax, const Vector3 & dir_parallax); 

Vector3 generateTranslationStereo( double max_parallax, const Vector3 & dir_parallax);

Vector3 generateTranslationSideways( double max_parallax, const Vector3 & dir_parallax); 

Vector3 generateTranslationOblique( double max_parallax, const Vector3 & dir_parallax); 

Vector3 generateOrbitalTranslation( double max_parallax, const Vector3 & dir_parallax); 

Matrix3 generateOrbitalRotation( double max_parallax, const Vector3 & dir_parallax, const Vector3& dir_trans); 
                        
};  // end of namespace UtilsTwoView

