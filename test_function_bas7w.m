clc;
clear;
close all;
format long
% 测试姿�?�更新函�?(AttitudeUpdate_single_sample.m)

% 包含路径,若此代码实效，手动添�?
addpath('./data');
addpath('./INS_lib');
addpath('./Kalman_f');
addpath('./Att_init_lib');
%                                                                                m/s     �?        rad/s     s
load("alldata_800_45_1_0005_0_75_45_30.mat");              %理论弹道,命名格式,    alldata   出膛初�??  射角(俯仰)  出膛转�??  采样时间
A_Data = load("TEST_800_45_1_0005_0_75_45_30.mat");        %模拟传感�?,命名格式,     TEST    出膛初�??  射角(俯仰)  出膛转�??  采样时间 
B_SensorData = A_Data.Z_IMU;
D_N = size(B_SensorData,1);
D_T = 0.005;

% % 生成噪声和随机游�?
% r_walk = random_walk(D_T,D_N*D_T,D_N,0.0003);
% rw = r_walk(1:D_N,1);
% % rw = zeros(D_N,1);
% 
% noise_gyr = [0;mvnrnd(0,0.000005,D_N-1)]+rw;
% noise_gps = mvnrnd(0,0.02,D_N);
% noise_acc = mvnrnd(0,0.000001,D_N)+rw;
% 
% % noise_gyr = zeros(D_N,1);
% % noise_gps = zeros(D_N,1);
% % noise_acc = zeros(D_N,1);
% 
% D_ACC = [B_SensorData(:,4)+noise_acc,B_SensorData(:,5)+noise_acc,B_SensorData(:,6)+noise_acc]';
% D_GYR = [B_SensorData(:,1)+noise_gyr,B_SensorData(:,2)+noise_gyr,B_SensorData(:,3)+noise_gyr]';
% GPS_Y = ([Y(:,1)+noise_gps,Y(:,2)+noise_gps,Y(:,3)+noise_gps]')';
% GPS_V = ([Y(:,4)+noise_gps/10,Y(:,5)+noise_gps/10,Y(:,6)+noise_gps/10]')';
% 
% figure(3);
% clf(3);
% plot3(GPS_Y(:,1),GPS_Y(:,2),GPS_Y(:,3),'K.');                              %绘制弹道
% 
% roll = atan(D_GYR(3,1)/D_GYR(1,1))/pi*180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%BAS初始化部�?1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j = 1:100
    r_walk = random_walk(D_T,D_N*D_T,D_N,0.0003);
    rw = r_walk(1:D_N,1);

    noise_gyr = [mvnrnd(0,0.000005,D_N)];
    noise_gps = mvnrnd(0,0.02,D_N);
    noise_acc = mvnrnd(0,0.000001,D_N)+rw;

    D_ACC = [B_SensorData(:,4)+noise_acc,B_SensorData(:,5)+noise_acc,B_SensorData(:,6)+noise_acc]';
    D_GYR = [B_SensorData(:,1)+noise_gyr,B_SensorData(:,2)+noise_gyr,B_SensorData(:,3)+noise_gyr]';
    GPS_Y = ([Y(:,1)+noise_gps,Y(:,2)+noise_gps,Y(:,3)+noise_gps]')';
    GPS_V = ([Y(:,4)+noise_gps/10,Y(:,5)+noise_gps/10,Y(:,6)+noise_gps/10]')';

    roll = atan(D_GYR(3,1)/D_GYR(1,1))/pi*180;
    store(j,:) = roll;
    store2(j,:) = roll-75;
end


% err_pitch = sqrt(sum(store2(:,1).^2)/100);
err_roll = sqrt(sum(store2(:,1).^2)/100);
% err_yaw = sqrt(sum(store2(:,3).^2)/100);

% err_pitch2 = sum(abs(store2(:,1)))/100;
err_roll2 = sum(abs(store2(:,1)))/100;
% err_yaw2 = sum(abs(store2(:,3)))/100;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%数据显示部分
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);clf(1);
% subplot(3,1,1);hold on;
plot(store2(:,1),'k');legend('���');
ylabel('��������(��)');xlabel('ʵ�����');
% title("��̬���");
grid on;


