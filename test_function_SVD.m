clc;
clear;
close all;
format long
% 测试姿态更新函数(AttitudeUpdate_single_sample.m)

% 包含路径,若此代码实效，手动添加
addpath('./data');
addpath('./INS_lib');
addpath('./Kalman_f');
addpath('./Att_init_lib');
%                                                                                    m/s     。        rad/s     s
load("alldata_800_45_1_0005_0(2).mat");              %理论弹道,命名格式,    alldata   出膛初速  射角(俯仰)  出膛转速  采样时间
A_Data = load("TEST_800_45_1_0005_0(2).mat");        %模拟传感器,命名格式,     TEST    出膛初速  射角(俯仰)  出膛转速  采样时间 
B_SensorData = A_Data.Z_IMU;
D_N = size(B_SensorData,1);
D_T = 0.005;

C_yaw  =  [
            cos(pi/4) sin(pi/4)   0;
           -sin(pi/4) cos(pi/4)   0;
                0        0        1;
          ];%在弹道坐标系下改变射向

%             XYZ   XYZ                   ZXY  ZXY
% Z_IMU坐标系:东北天-右前上,Y坐标系为弹道坐标系:东北天-右前上
% D_ACC = [B_SensorData(:,4),B_SensorData(:,5),B_SensorData(:,6)]';          %按照算法顺序重排加速度三轴读数顺序 x ,  y  , z
% D_GYR = [B_SensorData(:,1),B_SensorData(:,2),B_SensorData(:,3)]';          %按照算法顺序重排陀螺仪三轴读数顺序pitch,row,yaw 
% GPS_Y = (C_yaw*[Y(:,3),Y(:,1),Y(:,2)]')';
% GPS_V = (C_yaw*[Y(:,6),Y(:,4),Y(:,5)]')';

% 生成噪声和随机游走
r_walk = random_walk(D_T,D_N*D_T,D_N,0.001);
w_noise = mvnrnd(0,0.001,D_N);
% rw = r_walk(1:D_N,1) + w_noise;
rw = w_noise;
% rw = 0;
% Z_IMU使用的惯导格式

% 添加噪声和随机游走
D_ACC = [B_SensorData(:,4)+rw,B_SensorData(:,5)+rw,B_SensorData(:,6)+rw]';
D_GYR = [B_SensorData(:,1)+rw,B_SensorData(:,2)+rw,B_SensorData(:,3)+rw]';
GPS_Y = (C_yaw*[Y(:,3)+w_noise,Y(:,1)+w_noise,Y(:,2)+w_noise]')';
GPS_V = (C_yaw*[Y(:,6)+w_noise,Y(:,4)+w_noise,Y(:,5)+w_noise]')';
%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84简化参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WGS84.a = 6371100;                                                         %长半轴
WGS84.b = 6371100;                                                         %短半轴
WGS84.R = 6371100;                                                         %平均半径
WGS84.f = 0;                                                               %扁率
WGS84.e = 0;                                                               %偏心率
WGS84.e2 =0;                                                               %第一偏心率平方
WGS84.ep2=0;                                                               %第二偏心率平方
WGS84.we =7.292115e-5;                                                     %地球自转角速率
WGS84.GM =3.986004418e+14;                                                 %地球引力为常数
% WGS84.ge=9.7803267715;                                                   %赤道重力加速度
WGS84.ge =9.80;                                                            %常规重力加速度
WGS84.gp =9.8321863685;                                                    %极地重力加速度
%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84简化参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Eulerbb(:,1) = [45/180*pi;0;0];                                              %初始欧拉角pitch,yaw,row(弧度制)
% qbb(:,1) = Euler2Quaternion(Eulerbb(1),Eulerbb(2),Eulerbb(3));                   %初始姿态四元数,东北天坐标系，旋转顺序(zxy)312
% Cbb(:,:,1) = Quaternion2ACM(qbb(:,1));        

qbb(:,1) = [1;0;0;0];
Cbb(:,:,1) = Quaternion2ACM(qbb(:,1));

Cbn(:,:,1) = eye(3);
qbn(:,1) = [1;0;0;0];
Euler(:,1) = [0;0;0];

vb(:,1) = [0;0;0];
vn = [GPS_V(:,1),GPS_V(:,2),GPS_V(:,3)]';
A = zeros(3,3);

XYZb(:,1) = [0;0;0];

for i = 2:D_N                                          
%--------------------------------1.姿态矩阵解算------------------------------
%     [qbb(:,i),Cbb(:,:,i)] = AttitudeUpdate_Subsample(D_GYR(:,i-1)',qbb(:,i-1),D_T);
    
    Delta_sita(:,i) = D_GYR(:,i-1)*D_T+(D_GYR(:,i)-D_GYR(:,i-1))/2*D_T;   
    qbb(:,i) = AttitudeUpdate_single_sample(qbb(:,i-1), Delta_sita(:,i-1),Delta_sita(:,i));
    Eulerbb(:,i) = Quaternion2Euler(qbb(:,i))/pi*180;
    Cbb(:,:,i) = Quaternion2ACM(qbb(:,i));
%--------------------------------2.速度更新---------------------------------
%     vb(:,i) = vb(:,i-1) + Cbb(:,:,i-1) * D_ACC(:,i-1) * D_T;
    vb(:,i) = VelocityUpdate_SVD(vb(:,i-1),D_T,Cbb(:,:,i),Cbb(:,:,i-1),D_ACC(:,i),D_ACC(:,i-1));
%--------------------------------3.位置更新---------------------------------
%     XYZb(:,i) = LocationUpdata3(XYZb(:,i-1),vb(:,i),vb(:,i-1),D_T,WGS84);    
    Beta(:,i) = vn(:,i) - vn(:,1);   
    A = A + Beta(:,i) * vb(:,i)';
    % svd
    [U,D,V] = svds(A);
    % best Att Matrix
    C = U*V';
    Euler(:,i) = m2att(C)*180/pi;  
end
% load('BAS.mat');
% plot(BAS.x_store(2,:));
% hold on;
% plot(Euler(end,:));

