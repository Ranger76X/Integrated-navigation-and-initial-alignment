clc;
clear;
close all;
format long
% 测试姿�?�更新函�?(AttitudeUpdate_single_sample.m)

% 包含路径,若此代码实效，手动添�?
addpath('./data');
addpath('./INS_lib');
addpath('./Kalman_f');
load("err_ekf.mat");
load("DATA.mat")
%                                                                                m/s     �?        rad/s     s
load("alldata_800_45_1_0005_0.mat");              %理论弹道,命名格式,    alldata   出膛初�??  射角(俯仰)  出膛转�??  采样时间
A_Data = load("TEST_800_45_1_0005_0.mat");        %模拟传感�?,命名格式,     TEST    出膛初�??  射角(俯仰)  出膛转�??  采样时间 
B_SensorData = A_Data.Z_IMU;
D_N = size(B_SensorData,1);
D_T = 0.005;

%                 XYZ                   XYZ 
%Z_IMU坐标系为东北天_xyz,算法的坐标系为北东地_yx(-z),
D_ACC = [B_SensorData(:,5),B_SensorData(:,4),-B_SensorData(:,6)]';         %按照算法顺序重排加�?�度三轴读数顺序 y ,  x  , -z
%D_GYR = deg2rad(B_SensorData(:,1:3)');
D_GYR = [B_SensorData(:,2),B_SensorData(:,1),-B_SensorData(:,3)]';         %按照算法顺序重排�?螺仪三轴读数顺序row,pitch,yaw
                                                                           %                             y    x    -z
% %生成噪声和随机游�?
% mvnrnd(0,0.001,D_N);
% r_walk = random_walk(D_T,D_N*D_T,D_N,0.0002);
% rw = r_walk(1:D_N,1);
% rw = 0;
% % 添加噪声和随机游�?
% D_ACC = [B_SensorData(:,5)+mvnrnd(0,1,D_N)+rw,B_SensorData(:,4)+mvnrnd(0,1,D_N)+rw,-B_SensorData(:,6)+mvnrnd(0,1,D_N)+rw]';
% %D_GYR = deg2rad(B_SensorData(:,1:3)');
% D_GYR = [B_SensorData(:,2)+mvnrnd(0,1,D_N)+rw,B_SensorData(:,1)+mvnrnd(0,1,D_N)+rw,-B_SensorData(:,3)+mvnrnd(0,1,D_N)+rw]';
% % 添加噪声和随机游�?
Euler2(1,:) = Y(:,8)'/180*pi;                                              %row(弧度�?)
Euler2(2,:) = Y(:,10)'/180*pi;                                             %pitch(弧度�?)
Euler2(3,:) = (Y(:,9)'+90)/180*pi;                                         %yaw(弧度�?)
for i = 1:D_N
    qbn2(:,i) = EulerToQuaternion(Euler2(:,i));
    Cbn2(:,:,i) = QuaternionToDCM(qbn2(:,i));
end                                                                           
                                                                                                      
% v(:,1) = Cbn0*[400;0;0]; 
% v(:,1) = Cbn2(:,:,1)*[400;0;0];
v(:,1) = [Y(1,6);Y(1,4);-Y(1,5)];

XYZ(:,1) = [0;0;0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%姿�?�初始化%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Euler(1,1)=0/180*pi;                                                       %初始欧拉角row(弧度�?)
Euler(2,1)=45/180*pi;                                                      %pitch(弧度�?)
Euler(3,1)=90/180*pi;                                                      %yaw(弧度�?)

qbn(:,1)=EulerToQuaternion(Euler);                                         %初始姿�?�四元数
Cbn0=QuaternionToDCM(qbn(:,1));                                            %初始方向余弦�?

BLH(:,1)=[0;0;0];                                                          %初始位置（纬rad，经rad，高m�?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%姿�?�初始化%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84椭球参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WGS84.a = 6378137.0;                                                       %长半�?
WGS84.b=6356752.3142;                                                      %短半�?
WGS84.R=6371100;                                                           %平均半径
WGS84.f = 1/298.257223563;                                                 %扁率
WGS84.e2=0.00669437999013;                                                 %第一偏心率平�?
WGS84.ep2=0.006739496742227;                                               %第二偏心率平�?
WGS84.we=7.292115e-5;                                                      %地球自转角�?�率
WGS84.GM=3.986004418e+14;                                                  %地球引力为常�?
% WGS84.ge=9.7803267715;                                                   %赤道重力加�?�度
WGS84.ge=9.80;                                                             %常规重力加�?�度
WGS84.gp=9.8321863685;                                                     %极地重力加�?�度
%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84椭球参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(球参�?)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WGS84.a = 6371100;                                                       %平均半径
% WGS84.b = 6371100;                                                       %平均半径
% WGS84.R = 6371100;                                                       %平均半径
% WGS84.f = 0;                                                             %扁率
% WGS84.e2 = 1;                                                            %第一偏心率平�?
% WGS84.ep2 = 1;                                                           %第二偏心率平�?
% WGS84.we = 7.292115e-5;                                                  %地球自转角�?�率
% WGS84.GM = 3.986004418e+14;                                              %地球引力为常�?
% WGS84.ge = 9.80;                                                         %赤道重力加�?�度
% WGS84.gp = 9.80;                                                         %极地重力加�?�度
% %%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(球参�?)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


for i = 2:D_N                                          
%--------------------------------1.姿�?�矩阵解�?-----------------------------
    Delta_sita=D_GYR(:,i)*D_T;                                             %上一历元角�?�度增量                            
    Delta_sita_pre=D_GYR(:,i-1)*D_T;                                       %上上�?历元角�?�度增量
    [qbn(:,i),qbb(:,i)] = AttitudeUpdate(qbn(:,i-1),v(:,i-1),BLH(:,i-1),Delta_sita_pre,Delta_sita,D_T,WGS84);    
%--------------------------------2.速度解算---------------------------------
    Deltav=D_ACC(:,i)*D_T;                                                 %上一时刻速度增量
%     Deltav_pre=D_ACC(:,i-1)*D_T;                                         %上上�?时刻速度增量
%     v(:,i) = VelocityUpdate(v(:,i-1),BLH(:,i-1),D_T,Cbn0,Deltav_pre,Deltav,Delta_sita_pre, Delta_sita,WGS84);
    v(:,i) = VelocityUpdate2(v(:,i-1),XYZ(:,i-1),D_T,Cbn2(:,:,i),Deltav,WGS84);
    
    Z_Err_v(i,1) = Y(i,6)' + v(1,i);%x轴�?�度误差
    Z_Err_v(i,2) = Y(i,4)' - v(2,i);%y轴�?�度误差
    Z_Err_v(i,3) = Y(i,5)' + v(3,i);%z轴�?�度误差
%     v(1,i) = v(1,i) - Z_Err_v(i,1)';%补偿
%     v(2,i) = v(2,i) + Z_Err_v(i,2)';%补偿
%     v(3,i) = v(3,i) - Z_Err_v(i,3)';%补偿
%--------------------------------3.位置解算---------------------------------
    BLH(:,i) = LocationUpdata(BLH(:,i-1),v(:,i),v(:,i-1),D_T,WGS84);
    XYZ(:,i) = LocationUpdata2(XYZ(:,i-1),v(:,i),D_T);
%---------------------------------更新--------------------------------------
    Cbn0=QuaternionToDCM(qbn(:,i));
%     Euler(:,i) = rad2deg(qua2eul(qbn(:,i)));                               %转换为姿态（角度制）
%     Euler_2(:,i) = rad2deg(DCMToEuler(Cbn0));                            %                                                 
%     qre(:,i) = a2qua(deg2rad([Y(i,10),Y(i,8),Y(i,9)]));
end
Euler(:,1) = rad2deg(Euler(:,1));

Z_Err(:,1) = Y(1:D_N-2,3)'-XYZ(1,3:end);
Z_Err(:,2) = Y(1:D_N-2,1)'-XYZ(2,3:end);
Z_Err(:,3) = Y(1:D_N-2,2)'-XYZ(3,3:end);

figure(1);

plot3(XYZ(1,:),XYZ(2,:),XYZ(end,:),'r*');
%axis equal;
hold on;
plot3(Y(:,3),Y(:,1),Y(:,2),'b.');
hold on;
title('弹道');

legend('IMU解算弹道','理论弹道');


figure(2);
title('Y轴�?�度');
hold on;
plot(v(2,:),'k');
hold on;
plot(Y(:,4),'b');
legend('IMU解算弹道','理论弹道');


figure(3);
title('Z轴�?�度');
hold on;
plot(v(3,:),'k');
hold on;
plot(-Y(:,5),'b');
legend('IMU解算弹道','理论弹道');

figure(4);
title('X轴�?�度');
hold on;
plot(v(1,:),'k');
hold on;
plot(-Y(:,6),'b');
legend('IMU解算弹道','理论弹道');
