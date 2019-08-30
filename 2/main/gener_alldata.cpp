#include <fstream>

//#include "../src/imu.h"
//#include "../src/utilities.h"

#include "../src/imu.h"
#include "../src/utilities.h"

std::vector<std::pair<Eigen::Vector4d,Eigen::Vector4d>> CreatePointsLines(std::vector<Eigen::Vector4d,
Eigen::aligned_allocator<Eigen::Vector4d>> &points)
{
    std::vector<std::pair<Eigen::Vector4d,Eigen::Vector4d>> lines;

    std::ifstream f;
    f.open("house_model/hous.txt");

    while(!f.eof())
    {
        std::string s;
        std::getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss<<s;
            double x,y,z;
            ss>>x;ss>>y;ss>>z;
            Eigen::Vector4d pt0(x,y,z,1);
            ss>>x;ss>>y;ss>>z;
            Eigen::Vector4d pt1(x,y,z,1);

            bool isHistoryPoint=false;
            for(int i=0;i<points.size();++i)
            {
                Eigen::Vector4d pt=points[i];
                if(pt==pt0)
                {
                    isHistoryPoint=true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt0);//传入的points不是历史点把pt0赋值给它

            isHistoryPoint=false;
            for(int i=0;i<points.size();++i)
            {
                Eigen::Vector4d pt=points[i];
                if(pt==pt1)
                {
                    isHistoryPoint=true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt1);

            lines.push_back(std::make_pair(pt0,pt1));//两点组成一条线
        }
    }
    //产生更多的三维点
    int n=points.size();
    for(int j=0;j<n;++j)
    {
        Eigen::Vector4d p=points[j]+Eigen::Vector4d(0.5,0.5,-0.5,0);
        points.push_back(p);
    }

    //保存点
    std::stringstream filename;
    filename<<"all_points.txt";
    save_points(filename.str(),points);
    return lines;
}


int main()
{
    //生成3d points
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> points;
    std::vector<std::pair<Eigen::Vector4d,Eigen::Vector4d>> lines;
    lines=CreatePointsLines(points);

    //IMU模型
    Param params;
    IMU imuGen(params);//IMU类的实例化

    //IMU数据
    std::vector<MotionData> imudata;//一个结构体
    std::vector<MotionData> imudata_noise;

    for(float t=params.t_start;t<params.t_end;)
    {
        MotionData data=imuGen.MotionModel(t);
        imudata.push_back(data);

        //增加imu noise
        MotionData data_noise=data;
        imuGen.addIMUnoise(data_noise);//此时IMU的bias均已更新
        imudata_noise.push_back(data_noise);//另存为

        t+=1.0/params.imu_frequency;//t1=t1+1/200;

    }
    //开始给IMU赋予初始值
    imuGen.init_velocity_=imudata[0].imu_velocity;
    imuGen.init_twb_=imudata.at(0).twb;
    imuGen.init_Rwb_=imudata.at(0).Rwb;
    save_Pose("imu_pose.txt",imudata);//IMU中q,t gyro,acc保存文件
    save_Pose("imu_pose_noise.txt",imudata_noise);

    //利用IMU的数据产生IMu的轨迹
    imuGen.testIMU("imu_pose.txt","imu_int_pose.txt");
    imuGen.testIMU("imu_pose_noise.txt","imu_int_pose_noise.txt");

    //相机pose
    std::vector<MotionData> camdata;
    for(float t=params.t_start;t<params.t_end;)
    {
        //MotionData imu=imuGen.MotionModel(t);
        MotionData imu=imuGen.MotionModel(t);
        MotionData cam;

        cam.timestamp=imu.timestamp;
        cam.Rwb=imu.Rwb*params.R_bc;//c.Rwb=i.Rwb*Rbc
        cam.twb=imu.twb+imu.Rwb*params.t_bc;//c.twb=i.twb+i.Rwb*tbc
        //Tcw=Twb*Tbc,t=Rwb*tbc+twb

        camdata.push_back(cam);
        t+=1.0/params.cam_frequency;

    }
    save_Pose("cam_pose.txt",camdata);
    save_Pose_asTUM("cam_pose_tum.txt",camdata);

    //图像中的点
    for(int n=0;n<camdata.size();++n)
    {
        MotionData data=camdata[n];
        Eigen::Matrix4d Twc=Eigen::Matrix4d::Identity();
        Twc.block(0,0,3,3)=data.Rwb;
        Twc.block(0,3,3,1)=data.twb;

        //遍历所有在视野中的特征点
        std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>
        points_cam;//三维点在相机系下的坐标
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>>
        features_cam;  // 对应的２维图像坐标
        for(int i=0;i<points.size();++i)
        {
            Eigen::Vector4d pw=points[i];//最后一位是feature id
            pw[3]=1;//最后一位改成齐次坐标
            Eigen::Vector4d pc1=Twc.inverse()*pw;//世界坐标转到相机坐标

            //z必须大于0,在相机坐标系前方
            if(pc1(2)<0) continue;
            Eigen::Vector2d obs(pc1(0)/pc1(2),pc1(1)/pc1(2));//归一化坐标
            
            points_cam.push_back(points[i]);
            features_cam.push_back(obs);
        }

        //保存点
        std::stringstream filename1;
        filename1<<"keyframe/all_points_"<<n<<".txt";
        save_features(filename1.str(),points_cam,features_cam);

    }



   // lines obs in image
    for(int n = 0; n < camdata.size(); ++n)
    {
        MotionData data = camdata[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
       //std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features_cam;  // 对应的２维图像坐标
        for (int i = 0; i < lines.size(); ++i) {
            std::pair< Eigen::Vector4d, Eigen::Vector4d > linept = lines[i];

            Eigen::Vector4d pc1 = Twc.inverse() * linept.first; // T_wc.inverse() * Pw  -- > point in cam frame
            Eigen::Vector4d pc2 = Twc.inverse() * linept.second; // T_wc.inverse() * Pw  -- > point in cam frame

            if(pc1(2) < 0 || pc2(2) < 0) continue; // z必须大于０,在摄像机坐标系前方

            Eigen::Vector4d obs(pc1(0)/pc1(2), pc1(1)/pc1(2),
                                pc2(0)/pc2(2), pc2(1)/pc2(2));
            //if(obs(0) < params.image_h && obs(0) > 0 && obs(1)> 0 && obs(1) < params.image_w)
            {
                features_cam.push_back(obs);
            }
        }

        // save points
        std::stringstream filename1;
        filename1<<"keyframe/all_lines_"<<n<<".txt";
        save_lines(filename1.str(),features_cam);
    }


    return 1;
}