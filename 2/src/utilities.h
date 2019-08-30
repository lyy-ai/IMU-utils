#ifndef IMUSIMWITHPOINTLINE_UTILITIES_H
#define IMUSIMWITHPOINTLINE_UTILITIES_H

#include "imu.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>
#include <fstream>

//保存3d点
//void save_points(std::string filename,std::vector<Eigen::Vector4d,Eigen::aligned_allpcator<Eigen::Vector4d>> points);
void save_points(std::string filename, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points);
//保存IMU系的数据pose
void save_Pose(std::string filename,std::vector<MotionData> pose);

//从文件中下载IMU数据
void LoadPose(std::string filename,std::vector<MotionData>& pose);

//保存相机的轨迹为TUM格式
void save_Pose_asTUM(std::string filename,std::vector<MotionData> pose);

//保存三维点和其在相机里的2d点
void save_features(std::string filename,
                   std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                   std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features);

// save line obs
void save_lines(std::string filename,
                std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features);

#endif