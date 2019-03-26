#include "dyros_red_controller/dynamics_manager.h"

using namespace Eigen;

MatrixXd mat_inv(MatrixXd mat)
{

    JacobiSVD<MatrixXd> svd(mat, ComputeThinU | ComputeThinV);
    return mat;
}

DynamicsManager::DynamicsManager(DataContainer &dc_global)
    : dc(&dc_global)
{
}

void DynamicsManager::DynamicsUpdateThread(void)
{
    std::future<MatrixXd> ret[10];
    while (ros::ok())
    {

        ros::Time start = ros::Time::now();
        MatrixXd mat1, mat2;
        for (int i = 0; i < 10; i++)
        {
            mat1 = MatrixXd::Random(40, 40);
            ret[i] = std::async(&mat_inv, mat1);
        }

        for (int i = 0; i < 10; i++)
        {
            mat2 = ret[i].get();
        }

        std::cout << "thread calc done : " << (ros::Time::now() - start).toNSec() / 1.0E+6 << std::endl;

        start = ros::Time::now();

        for (int i = 0; i < 10; i++)
        {
            mat1.Random(40, 40);
            JacobiSVD<MatrixXd> svd(mat1, ComputeThinU | ComputeThinV);
            //mat2 = mat1.inverse();
        }

        std::cout << "none thread calc done : " << (ros::Time::now() - start).toNSec() / 1.0E+6 << std::endl;
    }
}