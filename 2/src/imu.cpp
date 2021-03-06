#include "imu.h"
#include "utilities.h"


//欧拉角转旋转矩阵，IMU系到惯性系
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}

//欧拉角速率到IMu速率
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}


IMU::IMU(Param p): param_(p)
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}



void IMU::addIMUnoise(MotionData& data)
{
    std::random_device rd;//随机数
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0,1.0);

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov=param_.gyro_noise_sigma*Eigen::Matrix3d::Identity();
    data.imu_gyro=data.imu_gyro+gyro_sqrt_cov*noise_gyro/sqrt(param_.imu_timestep)+
    gyro_bias_;//g1=g0+cov*w/t^-0.5+bg

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( param_.imu_timestep ) + acc_bias_;

    //陀螺仪bias更新
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    gyro_bias_+=param_.gyro_bias_sigma*sqrt(param_.imu_timestep)*noise_gyro_bias;
    //bg1=bg0+E*t^-0.5*wg ,bias求导就是噪声
    data.imu_gyro_bias=gyro_bias_;//更新了

    //加速度更新
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    acc_bias_+=param_.acc_bias_sigma*sqrt(param_.imu_timestep)*noise_acc_bias;
    //ba1=ba0+E*t^-0.5*wa
    data.imu_acc_bias=acc_bias_;

}


//IMU运动模型
MotionData IMU::MotionModel(double t)
{

    MotionData data;
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;           // z轴做sin运动
    float K1 = 10;          // z轴的正弦频率是x，y的k1倍
    float K = M_PI/ 10;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    // translation
    // twb:  body frame in world frame
    Eigen::Vector3d position( ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);
    Eigen::Vector3d dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));              // position导数　in world frame
    double K2 = K*K;
    Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数

//    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
//    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

    Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t;
    return data;

}


/*

//利用生成的IMU数据及IMU动力学模型对数据进行积分，保存积分以后的轨迹
 void testIMU(std::string src,std::string dist)
 {
      std::vector<MotionData> imudata;//结构体
      LoadPose(src,imudata);//src的数据给结构体imudata

     std::ofstream save_points;
      save_points.open(dist);
      
     // double dt=param_.imu_timestep;
     
      double dt = param_.imu_timestep;
      Eigen::Vector3d Pwb=init_twb_;
      Eigen::Quaterniond Qwb(init_Rwb_);
      Eigen::Vector3d Vw=init_velocity_;
      Eigen::Vector3d gw(0,0,-9.81);//东北天
      Eigen::Vector3d temp_a;
      Eigen::Vector3d theta;

      for(int i=1;i<imudata.size();++i)
      {
          //imu动力学模型 欧拉积分
          MotionData imupose=imudata[i];
          Eigen::Quaterniond dq;
          Eigen::Vector3d dtheta_half=imupose.imu_gyro*dt/2.0;//0.5*w*dt

          dq.w()=1;
          dq.x()=dtheta_half.x();
          dq.y()=dtheta_half.y();
          dq.z()=dtheta_half.z();

          Eigen::Vector3d acc_w=Qwb*(imupose.imu_acc)+gw;
          //a_w=Qwb(ab-na)+gw
          Qwb=Qwb*dq;
          Vw=Vw+acc_w*dt;
          Pwb=Pwb+Vw*dt+0.5*dt*dt*acc_w;
          
          //IMU存了两次
           save_points<<imupose.timestamp<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;

      }

 }*/

//读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testIMU(std::string src, std::string dist)
{
    /*std::vector<MotionData>imudata;
    LoadPose(src,imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;*/
      std::vector<MotionData> imudata;//结构体
      LoadPose(src,imudata);//src的数据给结构体imudata

     std::ofstream save_points;
      save_points.open(dist);
      double dt = param_.imu_timestep;



    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for (int i = 1; i < imudata.size(); ++i) {
       
       /// imu 动力学模型 欧拉积分
        /*MotionData imupose= imudata[i];
        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half = imupose.imu_gyro * dt /2.0;//理想数据，没减去bias
        
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();

        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
        Qwb = Qwb * dq;
        Vw = Vw + acc_w * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;*/
    
       /// 中值积分 作业题目
        MotionData imupose_0= imudata[i-1];
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half_0 = imupose_0.imu_gyro * dt /2.0;

        MotionData imupose_1 = imudata[i];
        Eigen::Vector3d dtheta_half_1 = imupose_1.imu_gyro * dt /2.0;
        Eigen::Vector3d dtheta_half = 0.5*(dtheta_half_0+dtheta_half_1);
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();

        Eigen::Vector3d acc_w_0=Qwb * (imupose_0.imu_acc)+gw; 
        Qwb= Qwb * dq;
        Eigen::Vector3d acc_w_1=Qwb * (imupose_1.imu_acc)+gw; 
        Eigen::Vector3d acc_w=0.5*(acc_w_0+acc_w_1);

        Vw = Vw + acc_w * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;


        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points<<imupose_1.timestamp<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;

    }

    std::cout<<"test　end"<<std::endl;

}
