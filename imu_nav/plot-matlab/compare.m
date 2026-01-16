clc;clear;
%% 数据读取与预处理
% 目标数据格式 week sow lat(deg) lon(deg) height(m) roll(deg) pitch(deg) heading(deg) 
% 1.示例数据(参考与结果)
simu_res = readmatrix("../assets/results/sim_res.ASC");
simu_ref = Read_simu_ref("../assets/results/PureINS.bin");

% 2.实测数据
% 参考文件格式：week sow lat lon height ... v_e v_n v_u roll pitch yaw
real_ref_tmp = readtable("../assets/results/IEproject7.ref", 'HeaderLines', 24,'FileType', 'text');
real_ref = real_ref_tmp(:, [1 2 3 4 5 20 19 21 24 23 22]);  
real_ref = table2array(real_ref);
real_ref(:,8) = -real_ref(:,8);
real_res = readmatrix("../assets/results/pure_imu_group33.ASC");
real_res_zupt = readmatrix("../assets/results/pure_imu_group33_zupt.ASC");


%% 示例数据比较
% plot_error(simu_ref, simu_res, 'SaveFigs', true, 'DPI', 600);
% plot_NED(simu_ref,simu_res,'SaveFigs', true, 'DPI', 600);

%% 实测数据比较(无零偏修正)
% plot_error(real_ref,real_res,'SaveFigs', true, 'DPI', 900);
% plot_NED(real_ref,real_res,'SaveFigs', true, 'DPI', 900);

%% 实测数据比较(含零速修正)
% plot_error(real_res,real_res_zupt,'SaveFigs', true, 'DPI', 900);
plot_NED(real_res_zupt,'SaveFigs', true, 'DPI', 900);
