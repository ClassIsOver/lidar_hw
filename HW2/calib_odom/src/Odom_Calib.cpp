#include "../include/calib_odom/Odom_Calib.hpp"


//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        //TODO: 构建超定方程组
        // Ai
        Eigen::MatrixXd<3,9,double> A_append.
        A_append.setZero();
        A_append.block<1,3>(0,0) = Odom.transpose();
        A_append.block<1,3>(1,3) = Odom.transpose();
        A_append.block<1,3>(2,6) = Odom.transpose();

        // bi
        Eigen::MatrixXd<3,1,double> b_append = scan;

        // Append Ai, bi to A and b
        A.conservativeResize(3 * now_len + 3, Eigen::NoChange);
        b.conservativeResize(3 * now_len + 3, Eigen::NoChange);
        A.block<3,9>(3 * now_len) = A_append;
        b.block<3,1>(3 * now_len) = b_append;
        //end of TODO
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;

    //TODO: 求解线性最小二乘
    corrected_matrix = (A.transpose() * A).inverse() * A.transpose() * b;
    //end of TODO

    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
