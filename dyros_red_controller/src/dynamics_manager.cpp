#include "dyros_red_controller/dynamics_manager.h"

using namespace Eigen;

std::mutex mtxd;
MatrixXd mat_inv(MatrixXd mat)
{

    JacobiSVD<MatrixXd> svd(mat, ComputeThinU | ComputeThinV);
    return mat;
}

DynamicsManager::DynamicsManager(DataContainer &dc_global) : dc(dc_global)
{
}

void DynamicsManager::dynamicsThread(void)
{
}

void DynamicsManager::testThread()
{
    int testval = 500;
    std::future<MatrixXd> ret[testval];

    while (ros::ok())
    {

        ros::Time start = ros::Time::now();
        MatrixXd mat1, mat2;
        for (int i = 0; i < testval; i++)
        {
            mat1 = MatrixXd::Random(40, 40);
            ret[i] = std::async(&mat_inv, mat1);
        }

        for (int i = 0; i < testval; i++)
        {
            mat2 = ret[i].get();
        }
        double d1, d2;
        d1 = (ros::Time::now() - start).toNSec() / 1.0E+6;

        //std::cout << "thread calc done : " << (ros::Time::now() - start).toNSec() / 1.0E+6 << std::endl;

        start = ros::Time::now();

        for (int i = 0; i < testval; i++)
        {
            mat1.Random(40, 40);
            JacobiSVD<MatrixXd> svd(mat1, ComputeThinU | ComputeThinV);
            //mat2 = mat1.inverse();
        }
        d2 = (ros::Time::now() - start).toNSec() / 1.0E+6;
        //std::cout << "none thread calc done : " << (ros::Time::now() - start).toNSec() / 1.0E+6 << std::endl;
        //std::cout << "dcl test : " << dc.check << std::endl;
        mtxd.lock();
        //erase();

        mvprintw(17, 10, "single thread : %8.4f    multi thread : %8.4f               ", d2, d1);

        mtxd.unlock();
        if (!(getch() == -1))
        {
            break;
        }
    }
}