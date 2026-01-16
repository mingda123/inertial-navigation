#include <iostream>
#include <string>
#include <vector>
#include "fileLoader.h"

namespace IMU
{
    FileLoader::FileLoader(const char *filename, bool flag)
        : flag(flag)
    {
        if (!flag)
            open(filename);
        else
            openBinary(filename);
    }

    FileLoader::~FileLoader()
    {
        file.close();
    }

    bool FileLoader::open(const char *filename)
    {
        memset(&imu_cur, 0, sizeof(IMUData));
        memset(&imu_last, 0, sizeof(IMUData));

        file.open(filename);

        if (!file.is_open())
        {
            std::cerr << "Error: Cannot open file " << filename << std::endl;
            return false;
        }
        return true;
    }

    bool FileLoader::openBinary(const char *filename)
    {
        file.open(filename, std::ios::binary);
        if (!file.is_open())
        {
            std::cerr << "Error: Cannot open binary file " << filename << std::endl;
            return false;
        }
        return true;
    }

    /*
     * @brief 重新打开文件
     */
    bool FileLoader::reset(const char *filename)
    {
        // 关闭当前文件
        if (file.is_open())
        {
            file.close();
        }

        // 重置内部状态
        imu_last = IMUData();
        imu_cur = IMUData();

        // 打开新文件
        return open(filename);
    }

    /*
     * @breif 读取单行数据
     * @param type 数据对应IMU型号
     * @param frq 采样率
     * @return 是否读取成功
     */
    bool FileLoader::read(int type, double frq)
    {
        std::string line;
        // 文件结束或读取失败/空行/非RAWIMUSA数据
        if (!getline(file, line) || line.empty() || line.find("IMU") == std::string::npos)
        {
            return false;
        }

        // 移除*后面的内容
        size_t checksum_pos = line.find('*');
        std::string data_line = (checksum_pos != std::string::npos) ? line.substr(0, checksum_pos) : line;

        // 分割数据
        std::vector<std::string> tokens;
        std::stringstream ss(data_line);
        std::string token;
        while (getline(ss, token, ','))
        {
            tokens.push_back(token);
        }

        double accel_scale = ACC_SCALE[type];
        double gyro_scale = GYRO_SCALE[type];
        std::string gpstime[2] = {tokens[1], tokens[2].substr(0, 9)};
        // 根据首字段确定数据开始顺序
        int start_idx = 5;
        if (tokens[0] == "%RAWIMUSA")
        {
            start_idx = 5;
        }
        else if (tokens[0] == "%RAWIMUSXA")
        {
            start_idx = 7;
        }
        else if (tokens[0] == "#RAWIMUA")
        {
            start_idx = 12;
            gpstime[0] = tokens[5];
            gpstime[1] = tokens[6];
            gyro_scale *= M_PI / 180.0; // 度转弧度
        }
        else
        {
            return false;
        }

        // 解析数据
        imu_cur.gpstime.week = stoi(gpstime[0]);
        imu_cur.gpstime.sow = stod(gpstime[1]);

        imu_cur.frq = frq;
        imu_cur.dvel[2] = stod(tokens[start_idx]) * accel_scale;
        imu_cur.dvel[1] = -stod(tokens[start_idx + 1]) * accel_scale;
        imu_cur.dvel[0] = stod(tokens[start_idx + 2]) * accel_scale;
        imu_cur.dangel[2] = stod(tokens[start_idx + 3]) * gyro_scale;
        imu_cur.dangel[1] = -stod(tokens[start_idx + 4]) * gyro_scale;
        imu_cur.dangel[0] = stod(tokens[start_idx + 5]) * gyro_scale;

        return true;
    }

    bool FileLoader::readBinary(int type, double frq)
    {
        struct BinaryIMURecord
        {
            double time;
            double gyro[3];
            double accel[3];
        };

        BinaryIMURecord record;

        if (!file.read(reinterpret_cast<char *>(&record), sizeof(record)))
        {
            return false; // 读取失败或到达文件末尾
        }

        imu_cur.gpstime.sow = record.time;
        imu_cur.frq = frq;
        imu_cur.dt = 1.0 / frq;

        imu_cur.dangel[0] = record.gyro[0];
        imu_cur.dangel[1] = record.gyro[1];
        imu_cur.dangel[2] = record.gyro[2];

        imu_cur.dvel[0] = record.accel[0];
        imu_cur.dvel[1] = record.accel[1];
        imu_cur.dvel[2] = record.accel[2];

        return true;
    }

    /*
     * @brief 读取时间间隔T内的均值
     * @param T 时间间隔(<0：单历元；0：所有数据；>0：T时间间隔)
     * @param type 数据对应IMU型号  (注意：-1 表述自定义数据)
     * @return 一组IMU数据
     */
    IMUData FileLoader::readSingleInT(int type, double frq, double T)
    {
        IMUData mean_imu; // 平均数据
        GPSTime start_time;
        double total_time = 0.;
        int sample_counts = 0;
        while (!file.eof())
        {
            imu_last = imu_cur;
            if (!flag)
            {
                if (!read(type, frq))
                    continue;
            }
            else
            {
                if (!readBinary(type, frq))
                    continue;
            }
            // 默认单历元
            if (T < 0)
            {
                imu_cur.dt = 1 / frq;
                return imu_cur;
            }

            // 累加数据
            sample_counts++;
            mean_imu.dangel += imu_cur.dangel;
            mean_imu.dvel += imu_cur.dvel;
            if (mean_imu.frq < 1e-10) // 起始
            {
                mean_imu.dt = 0.;
                start_time = imu_cur.gpstime;
                mean_imu.frq = imu_cur.frq;
                mean_imu.gpstime = imu_cur.gpstime;
            }
            total_time = (imu_cur.gpstime.week - start_time.week) * 86400 + (imu_cur.gpstime.sow - start_time.sow);
            mean_imu.dt = sample_counts / frq;

            // 达到时间间隔
            if (mean_imu.dt >= T && T > 0)
                break;
        }
        return mean_imu;
    }

    /*
     * @brief 读取文件中所有数据(滑动窗口取平均)
     * @param T 时间间隔(默认<0：单历元；0：所有数据；>0：T时间间隔)
     * @return 文件全部IMU数据
     */
    bool FileLoader::readAllInT(std::vector<IMUData> &datas, int type, double frq, double T)
    {
        while (!file.eof())
        {
            IMUData tmp = readSingleInT(type, frq, T);
            datas.push_back(tmp);
        }
        if (datas.empty())
            return false;
        return true;
    }

    /*
     * @brief 检查是否到达文件末尾
     */
    bool FileLoader::isEndOfFile()
    {
        if (!file.is_open())
        {
            return true; // 文件未打开，视为已到末尾
        }

        // 检查文件流状态
        return file.eof();
    }
}