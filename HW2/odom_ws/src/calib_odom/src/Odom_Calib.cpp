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

    if(now_len < INT_MAX && 3 * now_len + 3 <= data_len)
    {
        //TODO: 构建超定方程组
        // A
        A.block<1,3>(3 * now_len    , 0) = Odom.transpose();
        A.block<1,3>(3 * now_len + 1, 3) = Odom.transpose();
        A.block<1,3>(3 * now_len + 2, 6) = Odom.transpose();
        // b
        b.block<3,1>(3 * now_len, 0) = scan;
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
    std::cout << "begin solve!" <<  std::endl;
    Eigen::Map<Eigen::Matrix<double, 9, 1> > col_matrix(correct_matrix.data());
    col_matrix = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    std::cout << "solve finish!" <<  std::endl;
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
