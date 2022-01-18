#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>


#include "ceresSolver.h"

#include "ceres/ceres.h"


using namespace std; 


namespace CeresSolver
{


    class ReprojectionError {
     public:
      ReprojectionError(
            const Eigen::Matrix<double, 3, 4>& projection_matrix,
            const Eigen::Vector2d& feature)
            : projection_matrix_(projection_matrix), feature_(feature) {}

      template <typename T>
      bool operator()(const T* input_point, T* reprojection_error) const {
            Eigen::Map<const Eigen::Matrix<T, 4, 1> > point(input_point);

            // Multiply the point with the projection matrix, then perform homogeneous
            // normalization to obtain the 2D pixel location of the reprojection.

            const Eigen::Matrix<T, 2, 1> reprojected_pixel =  (projection_matrix_ * point).hnormalized();
            // Reprojection error is the distance from the reprojection to the observed
            // feature location.
            reprojection_error[0] = feature_[0] - reprojected_pixel[0]; 
            reprojection_error[1] = feature_[1] - reprojected_pixel[1];

            return true;
      }
      static ceres::CostFunction * Create(const Eigen::Matrix<double, 3, 4>& projection_matrix_, 
            const Eigen::Vector2d& feature_) {
                return (new ceres::AutoDiffCostFunction<ReprojectionError, 2,4>
                    (new ReprojectionError(projection_matrix_,feature_)));
        }

     private:
        const Eigen::Matrix<double, 3, 4>& projection_matrix_;
        const Eigen::Vector2d& feature_;
    };


    Eigen::Vector3d Triangulate(std::vector<std::pair
        <Eigen::Matrix<double, 3, 4>,Eigen::Vector2d>>&  datas, 
        Eigen::Vector3d & init_x) {
            Eigen::Vector4d x;
            x << init_x,1;
            ceres::Problem problem;
            ceres::Solver::Options options;
            ceres::LossFunction* loss= nullptr;
            ceres::Solver::Summary summary;

            for(auto & data:datas) {
                problem.AddResidualBlock(ReprojectionError::Create(data.first,data.second),loss,&x[0]);
            }
            ceres::Solve(options,&problem, &summary);
            return x.hnormalized();
    }


}   // end of namespace ceres
