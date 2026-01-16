#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>

#include "COMMON/MyTime.h"

namespace IMU
{
    // 0:星网宇达XW-7681;1:Novatel SPAN-100C；2：CI-1230(*frq)
    const double ACC_SCALE[3] = {0.05 / pow(2, 15), 2.0e-8, 1.0 / 655360.0};
    const double GYRO_SCALE[3] = {0.1 / 3600 / pow(2, 8), 1.0e-9, 1 / 160849.543863};

    struct IMUData
    {
        GPSTime gpstime; // GPS周和周内秒
        double dt = 0.;  // 与上一次数据的时间差
        double frq = 0.; // 频率

        Eigen::Vector3d dangel = Eigen::Vector3d::Zero(); // 角增量(deg)        *frq = 角速度
        Eigen::Vector3d dvel = Eigen::Vector3d::Zero();   // 速度增量(m/s)      *frq = 加速度
    };

    class FileLoader
    {
    private:
        std::ifstream file;
        bool open(const char *filename);
        bool openBinary(const char *filename);
        bool read(int type, double frq); // 读取单历元数据
        bool readBinary(int type, double frq);

        IMUData imu_last, imu_cur; // 上一组/这一组IMU数据
        bool flag = false;         // 是否读取二进制文件

    public:
        FileLoader();
        FileLoader(const char *filename, bool flag = false);
        ~FileLoader();
        bool reset(const char *filename);

        IMUData readSingleInT(int type, double frq, double T = -1); // 读取滑动窗口内平均的IMU数据
        bool readAllInT(std::vector<IMUData> &datas, int type, double frq, double T = -1);

        bool isEndOfFile();
    };
}