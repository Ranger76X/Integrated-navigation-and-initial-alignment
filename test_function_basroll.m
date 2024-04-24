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

% 生成噪声和随机游�?
r_walk = random_walk(D_T,D_N*D_T,D_N,0.0003);
rw = r_walk(1:D_N,1);
% rw = zeros(D_N,1);

noise_gyr = [0;mvnrnd(0,0.000005,D_N-1)]+rw;
noise_gps = mvnrnd(0,0.02,D_N);
noise_acc = mvnrnd(0,0.000001,D_N)+rw;

% noise_gyr = zeros(D_N,1);
% noise_gps = zeros(D_N,1);
% noise_acc = zeros(D_N,1);

D_ACC = [B_SensorData(:,4)+noise_acc,B_SensorData(:,5)+noise_acc,B_SensorData(:,6)+noise_acc]';
D_GYR = [B_SensorData(:,1)+noise_gyr,B_SensorData(:,2)+noise_gyr,B_SensorData(:,3)+noise_gyr]';
GPS_Y = ([Y(:,1)+noise_gps,Y(:,2)+noise_gps,Y(:,3)+noise_gps]')';
GPS_V = ([Y(:,4)+noise_gps/10,Y(:,5)+noise_gps/10,Y(:,6)+noise_gps/10]')';

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
    Delta_sita(:,i) = D_GYR(:,i-1)*D_T+(D_GYR(:,i)-D_GYR(:,i-1))/2*D_T;   
    qbb(:,i) = AttitudeUpdate_single_sample(qbb(:,i-1), Delta_sita(:,i-1),Delta_sita(:,i));
    Eulerbb(:,i) = Quaternion2Euler(qbb(:,i))/pi*180;
    Cbb(:,:,i) = Quaternion2ACM(qbb(:,i));
%--------------------------------2.速度更新---------------------------------
    vb(:,i) = VelocityUpdate_SVD(vb(:,i-1),D_T,Cbb(:,:,i),Cbb(:,:,i-1),D_ACC(:,i),D_ACC(:,i-1));
%--------------------------------3.位置更新---------------------------------  
    Beta(:,i) = vn(:,i) - vn(:,1);   
end

tic;
for i = 1:5000
    A = A + Beta(:,i) * vb(:,i)';
    % svd
    [U,D,V] = svds(A);
    % best Att Matrix
    C = U*V';
    Euler(:,i) = m2att(C)*180/pi;
end
zt1 = 0;
zt1 = zt1+toc; 

zt2 = 0;
tic;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%BAS初始化部�?1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll = atan(D_GYR(3,1)/D_GYR(1,1))/pi*180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%BAS初始化部�?1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
noise1 = mvnrnd(0,0.02,1);
noise2 = mvnrnd(0,0.02,1);
N = 60;%�?要的观测矢量对数
BAS.eta = 0.9;
BAS.c = 5;                  %ratio between step and d0
BAS.step = 5;               %initial step set as the largest input range
BAS.n = 5000;                %iterations
BAS.k = 1;                  %space dimension
BAS.x = mod(0,360);        %intial value
BAS.xbest = BAS.x;
BAS.f = f5(BAS.x,N,vb,Beta,noise1,noise2);
BAS.fbest = BAS.f;
BAS.fbest_store = BAS.fbest;
BAS.x_store = [0;BAS.x;BAS.fbest];



for i = 1:BAS.n
    BAS.d0=BAS.step/BAS.c;
    BAS.dir=rands(BAS.k,1);
    BAS.dir=BAS.dir/(eps+norm(BAS.dir));    
    
    BAS.xleft=BAS.x+BAS.dir*BAS.d0;
    BAS.fleft=f5(BAS.xleft,N,vb,Beta,noise1,noise2);
    BAS.xright=BAS.x-BAS.dir*BAS.d0;
    BAS.fright=f5(BAS.xright,N,vb,Beta,noise1,noise2);   
    
    BAS.x=BAS.x-BAS.step*BAS.dir*sign(BAS.fleft-BAS.fright);
    
    BAS.f = f5(BAS.x,N,vb,Beta,noise1,noise2);
    
    %%%%%%%%%%
    if BAS.f<BAS.fbest
        BAS.xbest=BAS.x;
        BAS.fbest=BAS.f;
    end
    %%%%%%%%%%%
    BAS.x_store=cat(2,BAS.x_store,[i;BAS.x;BAS.f]);
    BAS.fbest_store=[BAS.fbest_store;BAS.fbest];
    phi(i) = BAS.xbest;
    phi2(i) = BAS.x;
    display([num2str(i),':xbest=[',num2str(BAS.xbest'),'],fbest=',num2str(BAS.fbest)])
    %%%%%%%%%%%
    if(BAS.fbest >= 0.4)
        BAS.step=BAS.step;
    else
        BAS.step=BAS.step*BAS.eta;
    end
end
zt2 = zt2+toc; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 数据显示部分
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(2);
clf(2);
plot([1:1:5000]*0.01,phi2(1,:));
hold on
plot([1:1:5000]*0.01,phi(1,:));
hold on;
plot([1:1:5000]*0.01,Euler(2,:));
legend('BAS','BASR','SVD');
title('������ʼ�����̬��׼���');

display([num2str(i),':real=[',num2str([45;75;-30]'),'],fbest=',num2str(f3_2([45,75,-30],N,vb,Beta))])
