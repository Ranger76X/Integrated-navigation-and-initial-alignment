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

%                 XYZ
% Z_IMU坐标系为东北天_xyz
% D_ACC = [B_SensorData(:,4),B_SensorData(:,5),B_SensorData(:,6)]';          %按照算法顺序重排加�?�度三轴读数顺序 x ,  y  , z
% D_GYR = [B_SensorData(:,1),B_SensorData(:,2),B_SensorData(:,3)]';          %按照算法顺序重排�?螺仪三轴读数顺序pitch,row,yaw                                                                         %                             x    y    z


% 生成噪声和随机游�?
r_walk = random_walk(D_T,D_N*D_T,D_N,0.0001);
w_noise = mvnrnd(0,0.0005,D_N);
rw = r_walk(1:D_N,1) + w_noise;
rw = w_noise;
% w_noise = 0;
% rw = 0;
% Z_IMU使用的惯导格�?

% 添加噪声和随机游�?
D_ACC = [B_SensorData(:,4)+rw,B_SensorData(:,5)+rw,B_SensorData(:,6)+rw]';
D_GYR = [B_SensorData(:,1)+rw,B_SensorData(:,2)+rw,B_SensorData(:,3)+rw]';
GPS_Y = [Y(:,3)+w_noise,Y(:,1)+w_noise,Y(:,2)+w_noise];

Ballistic_data = [Y(:,3),Y(:,1),Y(:,2)];                                   %模拟弹道
GPS_test = Generate_GPS(GPS_Y,100,200,D_N);                                %100Hz   GPS
GPS_interpolate = Interpolation_GPS(GPS_Y,Ballistic_data,100,200,D_N);     %

% 添加噪声和随机游�?
D_ACC = [Data(:,4),Data(:,5),Data(:,6)]';
D_GYR = [Data(:,1),Data(:,2),Data(:,3)]';

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%位姿初始�?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
XYZ(:,1) = [0;0;0];                                                        %位置
BLH(:,1) = [0;0;0];                                                        %纬经�?
v(:,1) = [Data(1,7);Data(1,8);Data(1,9)];                                  %速度
Delta_sita = [0;0;0];                                                      %角增�?
% Euler(:,1) = [45/180*pi;0/180*pi;0/180*pi];                                              %初始欧拉角pitch,yaw,row(弧度�?)
Euler(:,1) = [40/180*pi;145/180*pi;0/180*pi];                                              %初始欧拉角pitch,yaw,row(弧度�?)
qbn(:,1) = Euler2Quaternion(Euler(1),Euler(2),Euler(3));                   %初始姿�?�四元数,东北天坐标系，旋转顺�?(zxy)312
Cbn(:,:,1) = Quaternion2ACM(qbn(:,1));                                     %初始姿�?�旋转矩�?,东北天坐标系，旋转顺�?(zxy)312
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%位姿初始�?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%kalman初始�?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I33 = eye(3);
O33 = zeros(3,3);

K1.K_x = zeros(9,1);
K1.K_x_next = zeros(9,1);

K1.K_P = 999*180/pi*eye(9);
K1.K_P_next = 1*eye(9);
K1.K_Pc(1,1) = 0; 

K1.K_K = eye(9,6);
K1.K_Q = 10*eye(9);
K1.K_R = 180/pi*0.5*eye(6);
K1.f_c = Cbn(:,:,1) * D_ACC(:,1);
% f_c = D_ACC(:,1);
K1.K_F = F_t(WGS84.we,v(:,1),BLH(3,1),Cbn(:,:,1),K1.f_c,WGS84.R,WGS84.R,BLH(1,1));
K1.K_H = [eye(6),zeros(6,3)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%kalman初始�?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 2:D_N                                          
%--------------------------------1.姿�?�矩阵解�?------------------------------
    Delta_sita(:,i) = D_GYR(:,i-1)*D_T+(D_GYR(:,i)-D_GYR(:,i-1))/2*D_T;
    
    qbn(:,i) = AttitudeUpdate_single_sample(qbn(:,i-1), Delta_sita(:,i-1),Delta_sita(:,i));

    Cbn(:,:,i) = Quaternion2ACM(qbn(:,i));
    Euler(:,i) = Quaternion2Euler(qbn(:,i));
%--------------------------------2.速度更新---------------------------------
    v(:,i) = VelocityUpdate3(v(:,i-1),D_T,Cbn(:,:,i),Cbn(:,:,i-1),D_ACC(:,i),D_ACC(:,i-1));
%--------------------------------3.位置更新---------------------------------
    XYZ(:,i) = LocationUpdata3(XYZ(:,i-1),v(:,i),v(:,i-1),D_T,WGS84);
    XYZ2(:,i) = B_XYZ2XYZ([XYZ(2,i);XYZ(3,i);XYZ(1,i)],BLH(2,1),BLH(1,1),BLH(3,1),0,WGS84);
    BLH(:,i) = XYZ2LLA(XYZ2(:,i),WGS84);

end
roll2 = atan(D_GYR(3,2)/D_GYR(1,2))/pi*180
Euler = Euler/pi*180;
Err = Euler(3,2:end)' - Y(1:D_N-1,8);

% Z_Err(:,1) = Y(:,3)'-XYZ(1,:);   %x
% Z_Err(:,2) = Y(:,1)'-XYZ(2,:);   %y
% Z_Err(:,3) = Y(:,2)'-XYZ(3,:);   %z

Z_Err3(:,1) = GPS_Y(4:10:end-1,1)'-XYZ(1,5:10:end);   %x
Z_Err3(:,2) = GPS_Y(4:10:end-1,2)'-XYZ(2,5:10:end);   %y
Z_Err3(:,3) = GPS_Y(4:10:end-1,3)'-XYZ(3,5:10:end);   %z

figure(1);
plot3(XYZ(1,:),XYZ(2,:),XYZ(end,:),'r*');hold on;grid on;
% plot3(Y(:,3),Y(:,1),Y(:,2),'b.');hold on;grid on;
axis equal;
ylabel('y����о���(m)');xlabel('x����о���(m)');zlabel('z����о���(m)');
title('���Ե������');
legend('IMU����켣','ʵ�ʵ���');



% figure(2);
% subplot(3,1,1);hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,1),'k');hold on;grid on;
% ylabel('x�����(m)');xlabel('ʱ��(s)');
% title('x����Ե������');legend('λ�����');
% subplot(3,1,2);hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,2),'k');hold on;grid on;
% ylabel('y�����(m)');xlabel('ʱ��(s)');
% title('y����Ե������');legend('λ�����');
% subplot(3,1,3);hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,3),'k');hold on;grid on;
% ylabel('z�����(m)');xlabel('ʱ��(s)');
% title('z����Ե������');legend('λ�����');
% 
% figure(9);clf(9)
% subplot(3,1,1);hold on;
% plot([1:10:D_N]*0.005,Z_Err2(:,1),'r');legend('GPSλ�����');hold on;
% plot([1:10:D_N]*0.005,Z_Err(:,1),'k');legend('��ϵ���λ�����');hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,1),'b');legend('IMUλ�����');hold on;grid on;
% ylabel('x����λ�����(m)');xlabel('ʱ��(s)');
% title("x����λ�����");grid on;
% subplot(3,1,2);hold on;
% plot([1:10:D_N]*0.005,Z_Err2(:,1),'r');legend('GPSλ�����');hold on;
% plot([1:10:D_N]*0.005,Z_Err(:,1),'k');legend('��ϵ���λ�����');hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,1),'b');legend('IMUλ�����');hold on;grid on;
% ylabel('y����λ�����(m)');xlabel('ʱ��(s)');
% title("y����λ�����");grid on;
% subplot(3,1,3);hold on;
% plot([1:10:D_N]*0.005,Z_Err2(:,1),'r');legend('GPSλ�����');hold on;
% plot([1:10:D_N]*0.005,Z_Err(:,1),'k');legend('��ϵ���λ�����');hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,1),'b');legend('IMUλ�����');hold on;grid on;
% ylabel('z����λ�����(m)');xlabel('ʱ��(s)');
% title("z����λ�����");grid on;
% 
% figure(7);clf(7);
% subplot(3,1,1);hold on;
% plot([1:D_N]*0.005,Euler(1,:),'k');legend('�Ƕ�');
% ylabel('������(��)');xlabel('ʱ��(s)');title("����������");grid on;
% subplot(3,1,2);hold on;
% plot([1:1:D_N]*0.005,Euler(2,:),'k');legend('�Ƕ�');
% ylabel('�����(��)');xlabel('ʱ��(s)');title("���������");grid on;
% subplot(3,1,3);hold on;
% plot([1:1:D_N]*0.005,Euler(3,:)+60,'k');legend('�Ƕ�');
% ylabel('ƫ����(��)');xlabel('ʱ��(s)');title("ƫ��������");grid on;
% 
% figure(8);clf(8);
% plot(w_noise,'k');
% title("��˹����");grid on;