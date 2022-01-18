#pragma once

#include <Eigen/Core>

namespace NViewsTrian
{
                typedef Eigen::Matrix<double, 2, 1> Vector2;
                typedef Eigen::Matrix<double, 3, 1> Vector3;
                typedef Eigen::Matrix<double, 4, 1> Vector4;
                typedef Eigen::Matrix<double, 6, 1> Vector6;
               
               
                typedef Eigen::Matrix<double, 2, 2> Matrix2;
                typedef Eigen::Matrix<double, 3, 3> Matrix3;                
                typedef Eigen::Matrix<double, 4, 4> Matrix4;
                typedef Eigen::Matrix<double, 3, 4> Matrix34;
                

                struct PairObj
                {
                
                        int id1 = 0; 
                        int id2 = 0; 
                        Vector3 p1; 
                        Vector3 p2; 
                        Matrix3 F; 
                        
                        PairObj(){};
                        
                        PairObj(const Vector3 & p1,
                                const Vector3 & p2,
                                const Matrix3 & F, 
                                const int id1, 
                                const int id2)
                                : p1(p1), p2(p2), id1(id1), id2(id2), F(F) {};
                };  // end of PairObj struct

                /* Struct for constraints */
                struct Constr2View
                {
                        Eigen::Matrix2d F = Eigen::Matrix2d::Identity(); 
                        double b = 0; 
                        Eigen::Vector2d Fp2 = Eigen::Vector2d::Zero(); 
                        Eigen::Vector2d Fp1 = Eigen::Vector2d::Zero(); 
                        int id_1 = 0; 
                        int id_2 = 0;
                        Constr2View(){};
                };  // end of struct for constraints

}  // end of namespace NViewsTrian
