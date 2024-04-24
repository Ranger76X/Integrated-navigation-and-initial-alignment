clc;
clear;
close all;
format long
%测试姿态更新函数(AttitudeUpdate_qua.m)
%测试四元数算法
%


%包含路径,若此代码实效，手动添加
addpath('./data');
addpath('./INS_lib');

%                                                                                m/s     。        rad/s     s
load("alldata_800_45_1_001_0.mat");              %理论弹道,命名格式,    alldata   出膛初速  射角(俯仰)  出膛转速  采样时间                               
A_Data = load("TEST_800_45_1_001_0.mat");        %模拟传感器,命名格式,     TEST    出膛初速  射角(俯仰)  出膛转速  采样时间 
B_SensorData = A_Data.Z_IMU;
D_N = size(B_SensorData,1);
D_T = 0.01;

%                 XYZ
%Z_IMU坐标系为东北天_xyz
D_ACC = [B_SensorData(:,4),B_SensorData(:,5),B_SensorData(:,6)]';         %按照算法顺序重排加速度三轴读数顺序 x ,  y  , z
D_GYR = [B_SensorData(:,1),B_SensorData(:,2),B_SensorData(:,3)]';         %按照算法顺序重排陀螺仪三轴读数顺序pitch,row,yaw
                                                                          %                             x    y    z
XYZ(:,1) = [0;0;0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%姿态初始化%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Euler(1,1) = 45/180*pi;                                                     %初始欧拉角pitch(弧度制)
Euler(2,1) = 0/180*pi;                                                      %row(弧度制)
Euler(3,1) = 0/180*pi;                                                      %yaw(弧度制)
qbn(:,1) = Euler2Quaternion(Euler(1),Euler(2),Euler(3));                    %初始姿态四元数,东北天坐标系，旋转顺序(zxy)312
% Cbn(:,:,1) = ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%姿态初始化%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 2:D_N                                          
%--------------------------------1.姿态矩阵解算-----------------------------
    qbn(:,i) = AttitudeUpdate_qua(qbn(:,i-1),D_GYR,D_T);
    
    Euler(:,i) = Quaternion2Euler(qbn(:,i));
%     Cbn(:,:,i) = 
%--------------------------------2.速度更新--------------------------------
    Delta_v = D_GYR(:,i) * D_T;
%     VelocityUpdate2(v(:,i-1),XYZ(:,i-1),D_T,Cbn0,Delta_v,WGS84);
end
Euler = Euler/pi*180;
% Err = Euler(3,2:end)' - Y(1:D_N-1,8);
