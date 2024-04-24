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
% %                                                                                m/s     �?        rad/s     s
% load("alldata_800_45_1_0005_0(4).mat");              %理论弹道,命名格式,    alldata   出膛初�??  射角(俯仰)  出膛转�??  采样时间
% A_Data = load("TEST_800_45_1_0005_0(4).mat");        %模拟传感�?,命名格式,     TEST    出膛初�??  射角(俯仰)  出膛转�??  采样时间 

%                                                                                m/s     �?        rad/s     s
load("alldata_800_45_1_0005_0_75_45_30.mat");              %理论弹道,命名格式,    alldata   出膛初�??  射角(俯仰)  出膛转�??  采样时间
A_Data = load("TEST_800_45_1_0005_0_75_45_30.mat");        %模拟传感�?,命名格式,     TEST    出膛初�??  射角(俯仰)  出膛转�??  采样时间 
B_SensorData = A_Data.Z_IMU;
D_N = size(B_SensorData,1);
D_T = 0.005;

yaw = 0;

C_yaw  =  [
            cos(yaw) sin(yaw)   0;
           -sin(yaw) cos(yaw)   0;
                0        0        1;
          ];%在弹道坐标系下改变射�?,绕弹道Y轴旋�?,对应导航坐标系下的pitch矩阵

% 生成噪声和随机游�?
r_walk = random_walk(D_T,D_N*D_T,D_N,0.001);
w_noise = mvnrnd(0,0.008,D_N);
rw = w_noise;
% rw = r_walk(1:D_N,1) + w_noise;
% rw = r_walk(1:D_N,1);
rw = 0;
w_noise = 0;

%             XYZ   XYZ                   ZXY  ZXY
% Z_IMU坐标�?:东北�?-右前�?,Y坐标系为弹道坐标�?:东北�?-右前�?
% D_ACC = [B_SensorData(:,4),B_SensorData(:,5),B_SensorData(:,6)]';        %按照算法顺序重排加�?�度三轴读数顺序 x ,  y  , z
% D_GYR = [B_SensorData(:,1),B_SensorData(:,2),B_SensorData(:,3)]';        %按照算法顺序重排�?螺仪三轴读数顺序pitch,row,yaw 
% GPS_Y = (C_yaw*[Y(:,3),Y(:,1),Y(:,2)]')';
% GPS_V = (C_yaw*[Y(:,6),Y(:,4),Y(:,5)]')';
% % 添加噪声和随机游�?
D_ACC = [B_SensorData(:,4)+rw,B_SensorData(:,5)+rw,B_SensorData(:,6)+rw]';
D_GYR = [B_SensorData(:,1)+rw,B_SensorData(:,2)+rw,B_SensorData(:,3)+rw]';
% GPS_Y = (C_yaw*[Y(:,3)+w_noise,Y(:,1)+w_noise,Y(:,2)+w_noise]')';
% GPS_V = (C_yaw*[Y(:,6)+w_noise,Y(:,4)+w_noise,Y(:,5)+w_noise]')';
GPS_Y = (C_yaw*[Y(:,1)+w_noise,Y(:,2)+w_noise,Y(:,3)+w_noise]')';
GPS_V = (C_yaw*[Y(:,4)+w_noise,Y(:,5)+w_noise,Y(:,6)+w_noise]')';
figure(3);
clf(3);
plot3(GPS_Y(:,1),GPS_Y(:,2),GPS_Y(:,3),'K.');                              %绘制弹道
%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84�?化参�?)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WGS84.a = 6371100;                                                         %长半�?
WGS84.b = 6371100;                                                         %短半�?
WGS84.R = 6371100;                                                         %平均半径
WGS84.f = 0;                                                               %扁率
WGS84.e = 0;                                                               %偏心�?
WGS84.e2 =0;                                                               %第一偏心率平�?
WGS84.ep2=0;                                                               %第二偏心率平�?
WGS84.we =7.292115e-5;                                                     %地球自转角�?�率
WGS84.GM =3.986004418e+14;                                                 %地球引力为常�?
% WGS84.ge=9.7803267715;                                                   %赤道重力加�?�度
WGS84.ge =9.80;                                                            %常规重力加�?�度
WGS84.gp =9.8321863685;                                                    %极地重力加�?�度
%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84�?化参�?)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Eulerbb(:,1) = [45/180*pi;0;0];                                            %初始欧拉角pitch,yaw,row(弧度�?)
% qbb(:,1) = Euler2Quaternion(Eulerbb(1),Eulerbb(2),Eulerbb(3));             %初始姿�?�四元数,东北天坐标系，旋转顺�?(zxy)312
% Cbb(:,:,1) = Quaternion2ACM(qbb(:,1));        

qbb(:,1) = [1;0;0;0];
Cbb(:,:,1) = Quaternion2ACM(qbb(:,1));
Eulerbb(:,1) = [0;0;0];

Cbn(:,:,1) = eye(3);
qbn(:,1) = [1;0;0;0];
Euler(:,1) = [0;0;0];                                                      %初始欧拉角pitch,yaw,row(弧度�?)

vb(:,1) = [0;0;0];
vn = [GPS_V(:,1),GPS_V(:,2),GPS_V(:,3)]';
Beta(:,1) = [0;0;0];
Delta_sita(:,1) = [0;0;0];
A = 0;
for i = 2:D_N                                          
%--------------------------------1.姿�?�矩阵解�?------------------------------
%     [qbb(:,i),Cbb(:,:,i)] = AttitudeUpdate_Subsample(D_GYR(:,i-1)',qbb(:,i-1),D_T);    
    Delta_sita(:,i) = D_GYR(:,i-1)*D_T+(D_GYR(:,i)-D_GYR(:,i-1))/2*D_T;   
    qbb(:,i) = AttitudeUpdate_single_sample(qbb(:,i-1), Delta_sita(:,i-1),Delta_sita(:,i));
    Eulerbb(:,i) = Quaternion2Euler(qbb(:,i))/pi*180;
    Cbb(:,:,i) = Quaternion2ACM(qbb(:,i));
%--------------------------------2.速度更新------------------------------------
%     vb(:,i) = vb(:,i-1) + Cbb(:,:,i-1) * D_ACC(:,i-1) * D_T;
    vb(:,i) = VelocityUpdate_SVD(vb(:,i-1),D_T,Cbb(:,:,i),Cbb(:,:,i-1),D_ACC(:,i),D_ACC(:,i-1));
%--------------------------------3.位置更新------------------------------------  
    Beta(:,i) = vn(:,i) - vn(:,1);  
end
N = 60;
l = 1;
func = [0,0,0,0];
for i = 0:50
    for j = 1:360
        for k = 0:35
            func(l,:) = [i,j,k,f_test([i;j;k],N,vb,Beta)];
            l = l + 1;
        end
        l = l + 1;
    end
    l = l + 1;
end