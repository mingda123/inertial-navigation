#include <vector>
#include <iostream>
#include <string>
#include <iomanip>

#include "simpleini/SimpleIni.h"

#include "IMU/inerNav.h"
#include "IMU/fileLoader.h"
#include "IMU/fileSaver.h"

#include "COMMON/const_info.h"

using namespace std;
using namespace IMU;
using namespace Eigen;

// 定义ini文档对象
CSimpleIniA ini;

void demarcate_demo()
{
    InerNav XW_7681_inerNav;

    // 加速度计标定
    array<vector<IMUData>, 6> static_data_vec;
    FileLoader fileloader(ini.GetValue("demarcate", "input_file_xup", ""));
    fileloader.readAllInT(static_data_vec[0], 0, 100, 0);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_xdown", "")))
        fileloader.readAllInT(static_data_vec[1], 0, 100, 0);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_yup", "")))
        fileloader.readAllInT(static_data_vec[2], 0, 100, 0);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_ydown", "")))
        fileloader.readAllInT(static_data_vec[3], 0, 100, 0);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_zup", "")))
        fileloader.readAllInT(static_data_vec[4], 0, 100, 0);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_zdown", "")))
        fileloader.readAllInT(static_data_vec[5], 0, 100, 0);

    XW_7681_inerNav.acc.demarcate(static_data_vec);
    XW_7681_inerNav.acc.showInfo();

    // 进一步分析
    vector<IMUData> acc_data;
    // readAllRawInerDataInT("assets/demarcate/acc/X_up.ASC", acc_data, 0, 100);
    // readAllRawInerDataInT("assets/demarcate/acc/Y_up.ASC", acc_data, 0, 100);
    if (fileloader.reset("./assets/demarcate/acc/Z_up.ASC"))
        fileloader.readAllInT(acc_data, 0, 100);
    string XWoutfile_acc = "./assets/results/XW_7681_acc_demarcate_zup.csv";
    ofstream file1(XWoutfile_acc);
    if (!file1.is_open())
    {
        std::cerr << "无法创建文件: " << XWoutfile_acc << std::endl;
        return;
    }
    file1 << "pro_acc_x,pro_acc_y,pro_acc_z,raw_acc_x,raw_acc_y,raw_acc_z\n";
    for (const auto &data : acc_data)
    {
        Vector3d pro_data = data.dvel / data.dt;
        XW_7681_inerNav.acc.compensate(pro_data);
        file1 << pro_data(0) << "," << pro_data(1) << "," << pro_data(2) << "," << (data.dvel / data.dt)[0] << "," << (data.dvel / data.dt)[1] << "," << (data.dvel / data.dt)[2] << "\n";
    }

    // 陀螺仪标定
    array<vector<IMUData>, 6> dy_data_vec;
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_xpos", "")))
        fileloader.readAllInT(dy_data_vec[0], 0, 100);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_xneg", "")))
        fileloader.readAllInT(dy_data_vec[1], 0, 100);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_ypos", "")))
        fileloader.readAllInT(dy_data_vec[2], 0, 100);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_yneg", "")))
        fileloader.readAllInT(dy_data_vec[3], 0, 100);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_zpos", "")))
        fileloader.readAllInT(dy_data_vec[4], 0, 100);
    if (fileloader.reset(ini.GetValue("demarcate", "input_file_zneg", "")))
        fileloader.readAllInT(dy_data_vec[5], 0, 100);
    XW_7681_inerNav.gyro.demarcate(static_data_vec, dy_data_vec);
    XW_7681_inerNav.gyro.showInfo();

    // 误差补偿
    string XWoutfile_gyro = "./assets/results/XW_7681_gyro_demarcate_xRotate.csv";
    ofstream file2(XWoutfile_gyro);
    if (!file2.is_open())
    {
        std::cerr << "无法创建文件: " << XWoutfile_gyro << std::endl;
        return;
    }
    file2 << "pro_gryo_x,pro_gryo_y,pro_gryo_z,raw_gryo_x,raw_gryo_y,raw_gryo_z\n";
    for (const auto &data : dy_data_vec[0])
    {
        Vector3d pro_data = data.dangel / data.dt;
        XW_7681_inerNav.gyro.compensate(pro_data);
        file2 << pro_data(0) << "," << pro_data(1) << "," << pro_data(2) << "," << (data.dangel / data.dt)[0] << "," << (data.dangel / data.dt)[1] << "," << (data.dangel / data.dt)[2] << "\n";
    }
    // for (const auto &data : dy_data_vec[1])
    // {
    //     Vector3d pro_data = data.dangel / data.dt;
    //     XW_7681_inerNav.gyro.compensate(pro_data);
    //     file2 << pro_data(0) << "," << pro_data(1) << "," << pro_data(2) << "," << (data.dangel / data.dt)[0] << "," << (data.dangel / data.dt)[1] << "," << (data.dangel / data.dt)[2] << "\n";
    // }
}

void align_demo()
{
    InerNav Novatel_SPAN_100C_inerNav;
    FileLoader fileloader(ini.GetValue("align", "input_file", ""));
    vector<IMUData> data, data1s, datatotal;
    fileloader.readAllInT(data, 1, 200);
    if (fileloader.reset(ini.GetValue("align", "input_file", "")))
        fileloader.readAllInT(data1s, 1, 200, 1);
    if (fileloader.reset(ini.GetValue("align", "input_file", "")))
        fileloader.readAllInT(datatotal, 1, 200, 0);

    string SPANoutfile1 = "./assets/results/SPAN_100C_align.txt";
    ofstream file1(SPANoutfile1, ios::trunc);
    if (!file1.is_open())
    {
        std::cerr << "failed to create file: " << SPANoutfile1 << std::endl;
        return;
    }

    file1 << "<< start total\n";
    for (auto &raw_data : datatotal)
    {
        PVA pva;
        Novatel_SPAN_100C_inerNav.compensate(raw_data);
        Novatel_SPAN_100C_inerNav.staticCoarseAlign(30.531651244, raw_data, pva);
        file1 << raw_data.gpstime.week << " " << fixed << setprecision(3) << raw_data.gpstime.sow << " " << pva.att.yaw * (180 / PI) << " " << pva.att.pitch * (180 / PI) << " " << pva.att.roll * (180 / PI) << endl;
    }
    file1 << "<< end total\n";
    file1 << "<< start 1s\n";
    for (auto &raw_data : data1s)
    {
        PVA pva;
        Novatel_SPAN_100C_inerNav.compensate(raw_data);
        Novatel_SPAN_100C_inerNav.staticCoarseAlign(30.531651244, raw_data, pva);
        file1 << raw_data.gpstime.week << " " << fixed << setprecision(3) << raw_data.gpstime.sow << " " << pva.att.yaw * (180 / PI) << " " << pva.att.pitch * (180 / PI) << " " << pva.att.roll * (180 / PI) << endl;
    }
    file1 << "<< end 1s\n";
    file1 << "<< start 1epoch\n";
    for (auto &raw_data : data)
    {
        PVA pva;
        Novatel_SPAN_100C_inerNav.compensate(raw_data);
        Novatel_SPAN_100C_inerNav.staticCoarseAlign(30.531651244, raw_data, pva);
        file1 << raw_data.gpstime.week << " " << fixed << setprecision(3) << raw_data.gpstime.sow << " " << pva.att.yaw * (180 / PI) << " " << pva.att.pitch * (180 / PI) << " " << pva.att.roll * (180 / PI) << endl;
    }
    file1 << "<< end 1epoch\n";
    return;
}

void pureimu_simul_demo()
{
    InerNav CI_1230_inerNav;
    FileLoader fileloader(ini.GetValue("pureimu", "simu_input_file", ""), true);
    FileSaver filesaver(ini.GetValue("pureimu", "simu_output_file", ""));
    IMUData imu_pre, imu_cur;
    PVA pva_pre2[2], pva_cur;

    while (!fileloader.isEndOfFile())
    {
        imu_cur = fileloader.readSingleInT(-1, 1 / 0.005);
        if (imu_cur.gpstime.sow > 91620.000)
        {
            bool is_static = CI_1230_inerNav.isZeroVel(imu_cur);
            CI_1230_inerNav.update(pva_pre2, pva_cur, imu_pre, imu_cur);
            filesaver.saveSingleInT(pva_cur, imu_cur);
        }
        else
        {
            pva_cur.blh = Vector3d(23.1373950708 * M_PI / 180.0, 113.3713651222 * M_PI / 180.0, 2.175);
            pva_cur.vel_n = Vector3d::Zero();
            pva_cur.att.yaw = -75.7498049314083 * M_PI / 180.0;
            pva_cur.att.pitch = -2.14251290749072 * M_PI / 180.0;
            pva_cur.att.roll = 0.0107951084511778 * M_PI / 180.0;
            pva_cur.att.Euler2Quarternion();
            pva_cur.att.Euler2DCM();
            pva_pre2[0] = pva_cur;
            pva_pre2[1] = pva_cur;
        }

        imu_pre = imu_cur;
        memset(&imu_cur, 0, sizeof(IMUData));
        pva_pre2[1] = pva_pre2[0];
        pva_pre2[0] = pva_cur;
    }
}

void pureimu_real_demo()
{
    InerNav CI_1230_inerNav;
    FileLoader fileloader(ini.GetValue("pureimu", "real_input_file", ""));
    FileSaver filesaver(ini.GetValue("pureimu", "real_output_file", ""));

    IMUData imu_pre, imu_cur, imu_mean;
    PVA pva_pre2[2], pva_cur;
    double static_time = 0.;

    while (!fileloader.isEndOfFile())
    {
        imu_cur = fileloader.readSingleInT(2, 100);
        CI_1230_inerNav.compensate(imu_cur, true);

        if (imu_cur.gpstime.sow > 100775.00)
        {
            if (imu_cur.gpstime.sow <= 100775.0 + 60.0 * 5)
            {
                static_time += imu_cur.dt;
                imu_mean.dangel += imu_cur.dangel;
                imu_mean.dvel += imu_cur.dvel;
                imu_mean.dt += imu_cur.dt;
                imu_mean.frq = 1.0;
                imu_mean.gpstime = imu_cur.gpstime;
            }
            else
            {
                if (imu_mean.frq != 0)
                {
                    CI_1230_inerNav.staticCoarseAlign(pva_cur.blh(0), imu_mean, pva_cur);
                    cout << "aligned from " << imu_cur.gpstime.sow - static_time << " to " << imu_cur.gpstime.sow << ", static time: " << static_time << " s" << endl;
                    imu_mean = IMUData();
                    static_time = 0.;
                }
                else
                {
                    CI_1230_inerNav.update(pva_pre2, pva_cur, imu_pre, imu_cur);
                }
                filesaver.saveSingleInT(pva_cur, imu_cur);
            }
        }
        else
        {
            pva_cur.blh = Vector3d(30.5279729836 * DEG2RAD, 114.3557011348 * DEG2RAD, 19.659);
            pva_cur.vel_n = Vector3d::Zero();
            pva_pre2[0] = pva_cur;
            pva_pre2[1] = pva_cur;
        }

        // 更新前后历元状态
        imu_pre = imu_cur;
        memset(&imu_cur, 0, sizeof(IMUData));
        pva_pre2[1] = pva_pre2[0];
        pva_pre2[0] = pva_cur;
    }
}

void pureimu_real_zupt_demo()
{
    InerNav CI_1230_inerNav;
    FileLoader fileloader(ini.GetValue("pureimu", "real_input_file", ""));
    FileSaver filesaver(ini.GetValue("pureimu", "real_output_file_zupt", ""));

    IMUData imu_pre, imu_cur, imu_mean;
    PVA pva_pre2[2], pva_cur;

    double static_time = 0.;
    bool has_zupt = false;

    while (!fileloader.isEndOfFile())
    {
        imu_cur = fileloader.readSingleInT(2, 100);
        CI_1230_inerNav.compensate(imu_cur, true);
        bool is_static = CI_1230_inerNav.isZeroVel(imu_cur);

        if (imu_cur.gpstime.sow > 100775.00)
        {
            if (imu_cur.gpstime.sow <= 100775.0 + 60.0 * 5)
            {
                static_time += imu_cur.dt;
                imu_mean.dangel += imu_cur.dangel;
                imu_mean.dvel += imu_cur.dvel;
                imu_mean.dt += imu_cur.dt;
                imu_mean.frq = 1.0;
                imu_mean.gpstime = imu_cur.gpstime;
            }
            else
            {
                if (imu_mean.frq != 0)
                {
                    CI_1230_inerNav.staticCoarseAlign(pva_cur.blh(0), imu_mean, pva_cur);
                    cout << "aligned from " << imu_cur.gpstime.sow - static_time << " to " << imu_cur.gpstime.sow << ", static time: " << static_time << " s" << endl;
                    imu_mean = IMUData();
                    pva_pre2[0] = pva_cur;
                    pva_pre2[1] = pva_cur;
                    static_time = 0.;
                }
                else
                {
                    static_time = is_static ? static_time + imu_cur.dt : 0.;
                    if (is_static && static_time > 3.0)
                    {
                        if (!has_zupt)
                        {
                            cout << "zupt from " << imu_cur.gpstime.sow;
                        }
                        CI_1230_inerNav.update_with_zupt(pva_pre2, pva_cur, imu_pre, imu_cur);
                        has_zupt = true;
                    }
                    else
                    {
                        if (has_zupt)
                        {
                            cout << " to " << imu_cur.gpstime.sow << endl;
                            has_zupt = false;
                        }
                        CI_1230_inerNav.update(pva_pre2, pva_cur, imu_pre, imu_cur);
                    }
                }
                filesaver.saveSingleInT(pva_cur, imu_cur);
            }
        }
        else
        {
            pva_cur.blh = Vector3d(30.5279729836 * DEG2RAD, 114.3557011348 * DEG2RAD, 19.659);
            pva_cur.vel_n = Vector3d::Zero();
            pva_pre2[0] = pva_cur;
            pva_pre2[1] = pva_cur;
        }

        // 更新前后历元状态
        imu_pre = imu_cur;
        pva_pre2[1] = pva_pre2[0];
        pva_pre2[0] = pva_cur;
    }
}

int main()
{
    // 加载ini文件
    SI_Error rc;
    rc = ini.LoadFile("./config/pureImu.ini");
    if (rc < 0)
    {
        printf("load %s ini file failure\n", "./config/pureImu.ini");
        return -1;
    }

    int choic;
    cout << "--------------------------------" << endl;
    cout << "please chooic demo:" << endl;
    cout << "1. demarcate demo" << endl;
    cout << "2. align demo" << endl;
    cout << "3. pure imu simulate demo" << endl;
    cout << "4. pure imu real data demo" << endl;
    cout << "5. pure imu real data with zupt demo" << endl;
    cout << "if you want to exit, please input -1" << endl;
    cout << "--------------------------------" << endl;
    cout << "enter your choice (1-5):" << endl;
    cin >> choic;
    switch (choic)
    {
    case 1:
        demarcate_demo();
        break;
    case 2:
        align_demo();
        break;
    case 3:
        pureimu_simul_demo();
        break;
    case 4:
        pureimu_real_demo();
        break;
    case 5:
        pureimu_real_zupt_demo();
        break;
    default:
        break;
    }

    // system("pause");
    return 0;
}