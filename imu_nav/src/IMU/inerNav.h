#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <array>
#include <vector>

#include "fileLoader.h"

#include "COMMON/const_info.h"
#include "COMMON/MyTime.h"
#include "COMMON/Attitude.h"

namespace IMU
{
#define STATIC_THRESHOLD_ACCEL 0.5 // 静态判别加速度阈值(m/s^2) 0.2-0.5
#define STATIC_THRESHOLD_GYRO 0.5  // 静态判别角速度阈值(rad/s) 0.1-0.5

	struct PVA
	{
		Eigen::Vector3d blh = Eigen::Vector3d::Zero();
		Eigen::Vector3d vel_n = Eigen::Vector3d::Zero(); // n系速度(NED)
		Attitude att;									 // n系姿态(NED)
	};

	// 加速度计(与仪器相关)
	class Accer
	{
	private:
		Eigen::Vector3d bias;	// 零偏
		Eigen::Matrix3d Sa, Na; // 比例因子与交轴耦合矩阵
	public:
		Accer()
		{
			bias = Eigen::Vector3d::Zero();
			Sa = Eigen::Matrix3d::Zero();
			Na = Eigen::Matrix3d::Zero();
		}
		void setBias(const Eigen::Vector3d &b) { bias = b; }
		void demarcate(const std::array<std::vector<IMUData>, 6> &data);
		void compensate(Eigen::Vector3d &acc_data);
		void showInfo();
	};

	// 陀螺仪
	class Gyro
	{
	private:
		Eigen::Vector3d bias; // 角速度零偏
		Eigen::Matrix3d Sg;	  // 角速度比例因子矩阵

	public:
		Gyro()
		{
			bias = Eigen::Vector3d::Zero();
			Sg = Eigen::Matrix3d::Zero();
		}
		void setBias(const Eigen::Vector3d &b) { bias = b; }
		void demarcate(const std::array<std::vector<IMUData>, 6> &sta_data, const std::array<std::vector<IMUData>, 6> &dy_data);
		void compensate(Eigen::Vector3d &gyro_data);
		void showInfo();
	};

	// 惯性导航系统
	class InerNav
	{
	private:
		bool velUpdate(const PVA pva_pre2[2], PVA &pva_cur, const IMUData &imu_pre, IMUData &imu_cur);
		bool blhUpdate(const PVA &pva_pre, PVA &pva_cur, const IMUData &imu_pre, IMUData &imu_cur);
		bool attitudeUpdate(const PVA &pva_pre, PVA &pva_cur, const IMUData &imu_pre, IMUData &imu_cur);

	public:
		Accer acc;
		Gyro gyro;

	public:
		InerNav() {}

		bool compensate(IMUData &raw_data, bool need_trans = false);
		void staticCoarseAlign(const double lat, const IMUData &imudata, PVA &pva);
		bool isZeroVel(const IMUData &imu_cur); // 判断零速
		void update(const PVA pva_pre2[2], PVA &pva_cur, const IMUData &imu_pre, IMUData &imu_cur);
		void update_with_zupt(const PVA pva_pre2[2], PVA &pva_cur, const IMUData &imu_pre, IMUData &imu_cur);
	};

};
