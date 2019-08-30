
//#ifdef IMUSIM_PARAM_H
#ifndef IMUSIM_PARAM_H
#define IMUSIM_PARAM_H

#include <eigen3/Eigen/Core>

class Param
{
    public:
        Param();

        //自定义一些参数
        int imu_frequency=200;
        int cam_frequency=30;
        double imu_timestep=1./imu_frequency;
        double cam_timestep=1./cam_frequency;
        double t_start=0;
        double t_end=20;//截取前20s进行计算

        //noise
        double gyro_bias_sigma=1.0e-5;
        double acc_bias_sigma=0.0001;
        double gyro_noise_sigma=0.015;//rad/s
        double acc_noise_sigma=0.019;//m/(s^2)
        double pixel_noise=1;

        //相机参数
        double fx=460;
        double fy=460;
        double cx=255;
        double cy=255;
        double image_w=640;
        double image_h=640;
        
        //外参
        Eigen::Matrix3d R_bc;
        Eigen::Vector3d t_bc;
};

#endif



