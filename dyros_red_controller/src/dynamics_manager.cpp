#include "dyros_red_controller/dynamics_manager.h"

using namespace Eigen;

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
    int testval = 500;
    while (ros::ok())
    {

        MatrixXd mat1;
        ros::Time start = ros::Time::now();
        for (int i = 0; i < testval; i++)
        {
            mat1 = MatrixXd::Random(40, 40);
            JacobiSVD<MatrixXd> svd(mat1, ComputeThinU | ComputeThinV);
            //mat2 = mat1.inverse();
        }
        //std::cout << "none thread calc done : " << (ros::Time::now() - start).toNSec() / 1.0E+6 << std::endl;
        //std::cout << "dcl test : " << dc.check << std::endl;

        if ((!(getch() == -1)) || dc.shutdown)
        {
            dc.shutdown = true;
            move(17, 0);
            clrtoeol();
            mvprintw(17, 10, "thread calc end ");
            break;
        }
    }
}

void DynamicsManager::testThread()
{
    int testval = 10;
    std::future<MatrixXd> ret[testval];
    mtx.lock();
    mvprintw(16, 10, "Calc SVD %d times ", testval);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    mtx.unlock();

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
        mtx.lock();
        //erase();

        mvprintw(17, 10, "single thread : %8.4f ms   multi thread : %8.4f ms              ", d2, d1);

        mtx.unlock();
        if ((!(getch() == -1)) || dc.shutdown)
        {
            dc.shutdown = true;
            move(17, 0);
            clrtoeol();
            move(16, 0);
            clrtoeol();
            mvprintw(17, 10, "thread calc end ");
            break;
        }
    }
}