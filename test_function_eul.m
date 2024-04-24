clc;
clear;
close all;
format long
%测试姿态更新函数(AttitudeUpdate_eul.m)
%测试欧拉角积分-龙格库塔

%包含路径,若此代码实效，手动添加
addpath('./data');
addpath('./INS_lib');

%                                                                                m/s     。        rad/s     s
load("alldata_800_45_1_0005_0.mat");              %理论弹道,命名格式,    alldata   出膛初速  射角(俯仰)  出膛转速  采样时间
A_Data = load("TEST_800_45_1_0005_0.mat");        %模拟传感器,命名格式,     TEST    出膛初速  射角(俯仰)  出膛转速  采样时间 
%                                                                                m/s     。        rad/s     s
% load("alldata_800_45_1_001_0005_0.mat");              %理论弹道,命名格式,    alldata   出膛初速  射角(俯仰)  出膛转速  采样时间
% A_Data = load("TEST_800_45_1_001_0005_0.mat");        %模拟传感器,命名格式,     TEST    出膛初速  射角(俯仰)  出膛转速  采样时间 
B_SensorData = A_Data.Z_IMU;
D_N = size(B_SensorData,1);
D_T = 0.005;

%                 XYZ
%Z_IMU坐标系为东北天_xyz
D_ACC = [B_SensorData(:,4),B_SensorData(:,5),B_SensorData(:,6)]';         %按照算法顺序重排加速度三轴读数顺序 x ,  y  , z
D_GYR = [B_SensorData(:,1),B_SensorData(:,2),B_SensorData(:,3)]';         %按照算法顺序重排陀螺仪三轴读数顺序pitch,row,yaw
                                                                          %                             x    y    z
XYZ(:,1) = [0;0;0];
v(:,1) = [Y(1,6);Y(1,4);Y(1,5)];
VP(:,1) = [XYZ(:,1);v(:,1)];
%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84椭球参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WGS84.a = 6378137.0;                                                       %长半轴
WGS84.b=6356752.3142;                                                      %短半轴
WGS84.R=6371100;                                                           %平均半径
WGS84.f = 1/298.257223563;                                                 %扁率
WGS84.e2=0.00669437999013;                                                 %第一偏心率平方
WGS84.ep2=0.006739496742227;                                               %第二偏心率平方
WGS84.we=7.292115e-5;                                                      %地球自转角速率
WGS84.GM=3.986004418e+14;                                                  %地球引力为常数
% WGS84.ge=9.7803267715;                                                   %赤道重力加速度
WGS84.ge=9.80;                                                             %常规重力加速度
WGS84.gp=9.8321863685;                                                     %极地重力加速度
%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84椭球参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%姿态初始化%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Euler(1,1) = 45/180*pi;                                                     %初始欧拉角pitch(弧度制)
Euler(2,1) = 0/180*pi;                                                      %row(弧度制)
Euler(3,1) = 0/180*pi;                                                      %yaw(弧度制)
sita(:,1) = [45;0;0]/180*pi;
qbn(:,1) = Euler2Quaternion(Euler(1),Euler(2),Euler(3));                    %初始姿态四元数,东北天坐标系，旋转顺序(zxy)312
Cbn(:,:,1) = Quaternion2ACM(qbn(:,1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%姿态初始化%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 2:D_N                                          
%--------------------------------1.姿态矩阵解算-----------------------------
    Euler(:,i) = AttitudeUpdate_eul(Euler(:,i-1),(D_GYR(:,i-1)+D_GYR(:,i))/2,D_T);
    Cbn(:,:,i) = Quaternion2ACM(Euler2Quaternion(Euler(1,i),Euler(2,i),Euler(3,i)));
%     [yn(:,i),k1(:,i),check(:,i)] = AttitudeUpdate_eul(yn(:,i-1),D_GYR(:,i-1),D_T,n);
%--------------------------------2.速度更新--------------------------------    
    v(:,i) = VelocityUpdate3(v(:,i-1),XYZ(:,i-1),D_T,Cbn(:,:,i),Cbn(:,:,i-1),D_ACC(:,i),D_ACC(:,i-1),WGS84);
%--------------------------------3.位置更新--------------------------------
    XYZ(:,i) = LocationUpdata3(XYZ(:,i-1),v(:,i),v(:,i-1),D_T,WGS84);    
end
Euler = Euler/pi*180;
Z_Err(:,1) = Y(:,3)'-XYZ(1,:);   %x
Z_Err(:,2) = Y(:,1)'-XYZ(2,:);   %y
Z_Err(:,3) = Y(:,2)'-XYZ(3,:);   %z

% yn = yn/pi*180;
% Err = Euler(3,2:end)' - Y(1:D_N-1,8);

figure(1);
plot3(XYZ(1,:),XYZ(2,:),XYZ(end,:),'r*');
% axis equal;
hold on;
plot3(Y(:,3),Y(:,1),Y(:,2),'b.');
hold on;
title('弹道');
legend('IMU解算弹道','理论弹道');


figure(2);
title('Y轴速度');
hold on;
plot(v(2,:),'k');
hold on;
plot(Y(:,4),'b');
legend('IMU解算弹道','理论弹道');


figure(3);
title('Z轴速度');
hold on;
plot(v(3,:),'k');
hold on;
plot(Y(:,5),'b');
legend('IMU解算弹道','理论弹道');

figure(4);
title('X轴速度');
hold on;
plot(v(1,:),'k');
hold on;
plot(Y(:,6),'b');
legend('IMU解算弹道','理论弹道');

