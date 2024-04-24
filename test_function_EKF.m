clc;
clear;
close all;
format long
% 测试姿�?�更新函�?(AttitudeUpdate_single_sample.m)

% 包含路径,若此代码实效，手动添�?
addpath('./data');
addpath('./INS_lib');
addpath('./Kalman_f');

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
r_walk = random_walk(D_T,D_N*D_T,D_N,0.01);
w_noise1 = mvnrnd(0,0.02,D_N);
w_noise2 = mvnrnd(0,0.03,D_N);
w_noise3 = mvnrnd(0,0.04,D_N);
w_noise4 = mvnrnd(0,2,D_N);
rw = r_walk(1:D_N,1) + w_noise1;
% rw = w_noise;
rw = 0;
% Z_IMU使用的惯导格�?

% 添加噪声和随机游�?
D_ACC = [B_SensorData(:,4)+rw,B_SensorData(:,5)+rw,B_SensorData(:,6)+rw]';
D_GYR = [B_SensorData(:,1)+rw,B_SensorData(:,2)+rw,B_SensorData(:,3)+rw]';
GPS_Y = [Y(:,3)+w_noise1,Y(:,1)+w_noise2,Y(:,2)+w_noise3];
GPS_V = [Y(:,6)+w_noise1,Y(:,4)+w_noise1,Y(:,5)+w_noise1;];

Ballistic_data = [Y(:,3),Y(:,1),Y(:,2)];                                   %模拟弹道
GPS_test = Generate_GPS(GPS_Y,100,200,D_N);                                %100Hz   GPS
GPS_interpolate = Interpolation_GPS(GPS_Y,Ballistic_data,100,200,D_N);     %

% 添加噪声和随机游�?

% %%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84椭球参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WGS84.a = 6378137.0;                                                       %长半�?
% WGS84.b=6356752.3142;                                                      %短半�?
% WGS84.R=6371100;                                                           %平均半径
% WGS84.f = 1/298.257223563;                                                 %扁率
% WGS84.e2=0.00669437999013;                                                 %第一偏心率平�?
% WGS84.ep2=0.006739496742227;                                               %第二偏心率平�?
% WGS84.we=7.292115e-5;                                                      %地球自转角�?�率
% WGS84.GM=3.986004418e+14;                                                  %地球引力为常�?
% % WGS84.ge=9.7803267715;                                                   %赤道重力加�?�度
% WGS84.ge=9.80;                                                             %常规重力加�?�度
% WGS84.gp=9.8321863685;                                                     %极地重力加�?�度
% %%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84椭球参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
v(:,1) = [Y(1,6);Y(1,4);Y(1,5)];                                           %速度
Delta_sita = [0;0;0];                                                      %角增�?
Euler(:,1) = [45/180*pi;0/180*pi;0/180*pi];                                              %初始欧拉角pitch,yaw,row(弧度�?)
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
K1.K_Q = 1000*eye(9);
K1.K_R = 180/pi*100*eye(6);
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
%------------------------------卡尔曼滤�?------------------------------------
    K1.f_c = Cbn(:,:,i) * D_ACC(:,i);
%     f_c = D_ACC(:,i);
    
    [K1.K_F,K_F2] = F_t(WGS84.we,v(:,i),BLH(3,i),Cbn(:,:,i),K1.f_c,WGS84.R,WGS84.R,BLH(1,i));
    
    K1.K_x_next = (K1.K_F + eye(9)) * K1.K_x(:,i-1);
    
    K1.K_P_next = K1.K_F * K1.K_P(:,:,i-1) * K1.K_F' + K1.K_Q;
    
    K1.K_K = K1.K_P_next * K1.K_H'/(K1.K_H * K1.K_P_next * K1.K_H' + K1.K_R);
  
%     K1.K_Perr(:,i) = [Y(i-1,3) - XYZ(1,i);Y(i-1,1) - XYZ(2,i);Y(i-1,2) - XYZ(3,i)];
%     K1.K_Verr(:,i) = [Y(i-1,6) - v(1,i);Y(i-1,4) - v(2,i);Y(i-1,5) - v(3,i);];
%     K1.K_Perr(:,i) = [GPS_interpolate(i-1,1) - XYZ(1,i);GPS_interpolate(i-1,2) - XYZ(2,i);GPS_interpolate(i-1,3) - XYZ(3,i)];
%     K1.K_Perr(:,i) = [GPS_Y(i-1,1) - XYZ(1,i);GPS_Y(i-1,2) - XYZ(2,i);GPS_Y(i-1,3) - XYZ(3,i)];
    K1.K_Perr(:,i) = [GPS_interpolate(i-1,1) - XYZ(1,i);GPS_interpolate(i-1,2) - XYZ(2,i);GPS_interpolate(i-1,3) - XYZ(3,i)];
    K1.K_Verr(:,i) = [GPS_V(i-1,1) - v(1,i);GPS_V(i-1,2) - v(2,i);GPS_V(i-1,3) - v(3,i);];

%     K_Eerr(:,i) = anti_askew(logm(Cbn0(:,:,i)' * Y_Cbn0(:,:,i)));
    K1.K_y = [K1.K_Verr(:,i);K1.K_Perr(:,i)];
     
    K1.K_x(:,i) = K1.K_x_next + K1.K_K * (K1.K_y - K1.K_H * K1.K_x_next);
     
    K1.K_P(:,:,i) = (eye(9) - K1.K_K * K1.K_H) * K1.K_P_next;
    K1.K_P(:,:,i) = (K1.K_P(:,:,i) + K1.K_P(:,:,i)') / 2;
    
    K1.K_Pc(i-1) = sqrt(trace(K1.K_P(:,:,i)));
    
    XYZ(1:3,i) = XYZ(1:3,i) + K1.K_x(4:6,i);
    
    v(:,i) = v(:,i) + K1.K_x(1:3,i);
%-----------------------------卡尔曼滤�?-------------------------------------    
end

Euler = Euler/pi*180;
Err = Euler(3,2:end)' - Y(1:D_N-1,8);

Z_Err(:,1) = Y(4:10:end-1,3)'-XYZ(1,5:10:end);   %x
Z_Err(:,2) = Y(4:10:end-1,1)'-XYZ(2,5:10:end);   %y
Z_Err(:,3) = Y(4:10:end-1,2)'-XYZ(3,5:10:end);   %z

Z_Err2(:,1) = GPS_Y(4:10:end-1,1)-Y(4:10:end-1,3);   %x
Z_Err2(:,2) = GPS_Y(4:10:end-1,2)-Y(4:10:end-1,1);   %y
Z_Err2(:,3) = GPS_Y(4:10:end-1,3)-Y(4:10:end-1,2);   %z
% 
% Y(:,6)+w_noise/10,Y(:,4)+w_noise/10,Y(:,5)+w_noise/10;
Z_ErrV(:,1) = Y(4:10:end-1,6)'-v(1,5:10:end);   %x
Z_ErrV(:,2) = Y(4:10:end-1,4)'-v(2,5:10:end);   %y
Z_ErrV(:,3) = Y(4:10:end-1,5)'-v(3,5:10:end);   %z
Z_ErrV2(:,1) = GPS_V(4:10:end-1,1)-Y(4:10:end-1,6);   %x
Z_ErrV2(:,2) = GPS_V(4:10:end-1,2)-Y(4:10:end-1,4);   %y
Z_ErrV2(:,3) = GPS_V(4:10:end-1,3)-Y(4:10:end-1,5);   %z


figure(9);clf(9)
subplot(3,1,1);hold on;
plot([1:10:D_N]*0.005,Z_Err2(:,1),'r');hold on;
plot([1:10:D_N]*0.005,Z_Err(:,1),'-k');legend('GNSSλ�����','��ϵ���λ�����');
ylabel('x����λ�����(m)');xlabel('ʱ��(s)');
title("x����λ�����");grid on;
subplot(3,1,2);hold on;
plot([1:10:D_N]*0.005,Z_Err2(:,2),'r');hold on;
plot([1:10:D_N]*0.005,Z_Err(:,2),'-k');legend('GNSSλ�����','��ϵ���λ�����');
ylabel('y����λ�����(m)');xlabel('ʱ��(s)');
title("y����λ�����");grid on;
subplot(3,1,3);hold on;
plot([1:10:D_N]*0.005,Z_Err2(:,3),'r');hold on;
plot([1:10:D_N]*0.005,Z_Err(:,3),'-k');legend('GNSSλ�����','��ϵ���λ�����');
ylabel('z����λ�����(m)');xlabel('ʱ��(s)');
title("z����λ�����");grid on;

figure(11);clf(11)
subplot(3,1,1);hold on;
plot([1:10:D_N]*0.005,Z_ErrV2(:,1),'r');hold on;
plot([1:10:D_N]*0.005,Z_ErrV(:,1),'-k');legend('GNSS�ٶ����','��ϵ����ٶ����');
ylabel('x�����ٶ����(m/s)');xlabel('ʱ��(s)');
title("x�����ٶ����");grid on;
subplot(3,1,2);hold on;
plot([1:10:D_N]*0.005,Z_ErrV2(:,2),'r');hold on;
plot([1:10:D_N]*0.005,Z_ErrV(:,2),'-k');legend('GNSS�ٶ����','��ϵ����ٶ����');
ylabel('y�����ٶ����(m/s)');xlabel('ʱ��(s)');
title("y�����ٶ����");grid on;
subplot(3,1,3);hold on;
plot([1:10:D_N]*0.005,Z_ErrV2(:,3),'r');hold on;
plot([1:10:D_N]*0.005,Z_ErrV(:,3),'-k');legend('GNSS�ٶ����','��ϵ����ٶ����');
ylabel('z�����ٶ����(m/s)');xlabel('ʱ��(s)');
title("z�����ٶ����");grid on;

err_x2 = sqrt(sum(Z_Err2(:,1).^2)/1353);
err_y2 = sqrt(sum(Z_Err2(:,2).^2)/1353);
err_z2 = sqrt(sum(Z_Err2(:,3).^2)/1353);
err_x1 = sqrt(sum(Z_Err(:,1).^2)/1353);
err_y1 = sqrt(sum(Z_Err(:,2).^2)/1353);
err_z1 = sqrt(sum(Z_Err(:,3).^2)/1353);

figure(1);
plot3(XYZ(1,200:250),XYZ(2,200:250),XYZ(end,200:250),'r*');hold on;
% axis equal;
plot3(Y(200:250,3),Y(200:250,1),Y(200:250,2),'bo');hold on;
plot3(GPS_Y(200:250,1),GPS_Y(200:250,2),GPS_Y(200:250,3),'K.');hold on;
ylabel('y������');xlabel('x������');zlabel('z������');
% title('��ϵ����켣ͼ');
legend('��ϵ����켣','����ģ�͹켣','GNSS�켣');

figure(12);
plot3(XYZ(1,1:10:end),XYZ(2,1:10:end),XYZ(end,1:10:end),'r*');hold on;
% axis equal;
plot3(Y(1:10:end,3),Y(1:10:end,1),Y(1:10:end,2),'bo');hold on;
plot3(GPS_Y(1:10:end,1),GPS_Y(1:10:end,2),GPS_Y(1:10:end,3),'K.');hold on;
ylabel('y������');xlabel('x������');zlabel('z������');
% title('��ϵ����켣ͼ');
legend('��ϵ����켣','����ģ�͹켣','GNSS�켣');

% figure(2);
% title('Y轴�?�度');
% hold on;
% plot(v(2,:),'k');
% hold on;
% plot(Y(:,4),'b');
% legend('IMU解算弹道','理论弹道');


% figure(3);
% title('Z轴�?�度');
% hold on;
% plot(v(3,:),'k');
% hold on;
% plot(Y(:,5),'b');
% legend('IMU解算弹道','理论弹道');
% 
% figure(4);
% title('X轴�?�度');
% hold on;
% plot(v(1,:),'k');
% hold on;
% plot(Y(:,6),'b');
% legend('IMU解算弹道','理论弹道');

figure(7);clf(7);
subplot(3,1,1);hold on;
plot([1:D_N]*0.005,Euler(1,:),'k');legend('�Ƕ�');
ylabel('������(��)');xlabel('ʱ��(s)');
title("����������");grid on;
subplot(3,1,2);hold on;
plot([1:1:D_N]*0.005,Euler(2,:),'k');legend('�Ƕ�');
ylabel('�����(��)');xlabel('ʱ��(s)');
title("���������");grid on;
subplot(3,1,3);hold on;
plot([1:1:D_N]*0.005,Euler(3,:)+60,'k');legend('�Ƕ�');
ylabel('ƫ����(��)');xlabel('ʱ��(s)');
title("ƫ��������");grid on;
% 
% figure(8);clf(8);
% plot(w_noise1,'k');
% title("��˹����");grid on;