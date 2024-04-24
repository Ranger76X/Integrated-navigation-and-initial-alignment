clc;clear;close all;
addpath('./INS_lib');
addpath('./DATA');

load("RTK_att.mat");
load("RTK_lla.mat");
load("IMU_gyroacc.mat");
load("D_RTK_DATA.mat");
load("ODO_vel.mat");

D_N = size(IMU_gyroacc,1);
D_N_GPS = size(RTK_lla,1);
D_N_ODO = size(ODO_vel,1);
D_N_bia = 46455;
D_T = 0.004;

D_ACC = [-IMU_gyroacc(:,6),IMU_gyroacc(:,5),IMU_gyroacc(:,7)]';
D_GYR = [-IMU_gyroacc(:,3),IMU_gyroacc(:,2),IMU_gyroacc(:,4)]';
% D_GYR = [zeros(D_N,1),zeros(D_N,1),IMU_gyroacc(:,4)]';

D_ACC_T = IMU_gyroacc(:,1)';
D_GPS_lla = [RTK_lla(:,2),RTK_lla(:,3),RTK_lla(:,4)]';
D_GPS_V = [D_RTK_DATA(:,8),D_RTK_DATA(:,9),D_RTK_DATA(:,10)]';
D_GPS_T = D_RTK_DATA(:,1)';

[D_ACC_BIA,D_GYR_BIA] = IMU_cal_bia(D_ACC,D_GYR,D_N_bia);
D_ACC = D_ACC - D_ACC_BIA;
% D_GYR = D_GYR-[0;0;D_GYR_BIA(3)];
D_GYR = D_GYR - D_GYR_BIA;

Df_IMU = [D_ACC(:,1:5:end);D_GYR(:,1:5:end)];
Df_GPS = [D_GPS_lla(:,1:2:end);D_GPS_V(:,1:2:end)];
Df_GPS_T = D_GPS_T(:,1:2:end)-1.694253291e+18;
Df_ACC_T = D_ACC_T(:,1:5:end)-1.694253291e+18;
Df_data = [Df_IMU(:,1:end);Df_GPS(:,1:size(Df_IMU,2));Df_GPS_T(:,1:size(Df_IMU,2));Df_ACC_T(:,1:end)];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%����������(wgs84)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WGS84.a = 6371100;                                                        
% WGS84.b = 6371100;                                                         
% WGS84.R = 6371100;                                                         
% WGS84.f = 0;                                                               
% WGS84.e = 0;                                                               
% WGS84.e2 =0;                                                               
% WGS84.ep2=0;                                                               
% WGS84.we =7.292115e-5;                                                     
% WGS84.GM =3.986004418e+14;                                                                                                  
% WGS84.ge =9.80;                                                            
% WGS84.gp =9.8321863685;                                                    
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%����������(wgs84)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84椭球参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WGS84.a = 6378137.0;                                                       %长半�?
WGS84.b=6356752.3142;                                                      %短半�?
WGS84.f = 1/298.257223563;                                                 %扁率
WGS84.e2=0.00669437999013;                                                 %第一偏心率平�?
WGS84.ep2=0.006739496742227;                                               %第二偏心率平�?
WGS84.we=7.292115e-5;                                                      %地球自转角�?�率
WGS84.GM=3.986004418e+14;                                                  %地球引力为常�?
WGS84.ge=9.7803267715;                                                     %赤道重力加�?�度
WGS84.gp=9.8321863685;                                                     %极地重力加�?�度
%%%%%%%%%%%%%%%%%%%%%%%%%%地球模型(WGS84椭球参数)%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Eulerbb(:,1) = [RTK_att(1,1)/180*pi;RTK_att(2,1)/180*pi;RTK_att(3,1)/180*pi];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%������ʼ����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Eulerbb(:,1) = [0/180*pi;0/180*pi;-1/180*pi];
Eulerbb(:,1) = [1.028/180*pi;1.632/180*pi;-175.463/180*pi];
% Eulerbb(:,1) = [0/180*pi;0/180*pi;0/180*pi];
qbb(:,1) = Euler2Quaternion(Eulerbb(1),Eulerbb(2),Eulerbb(3));                   
Cbb(:,:,1) = Quaternion2ACM(qbb(:,1));                                                                                                            
v(:,1) = [0;0;0];                                           
Delta_sita = [0;0;0];   
XYZ(:,1) = [32.0264838190000;118.853121620000;0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%������ʼ����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 2:D_N                                         
%--------------------------------1.��̬����------------------------------
    Delta_sita(:,i) = D_GYR(:,i-1)*D_T+(D_GYR(:,i)-D_GYR(:,i-1))/2*D_T;   
    qbb(:,i) = AttitudeUpdate_single_sample(qbb(:,i-1), Delta_sita(:,i-1),Delta_sita(:,i));
    Eulerbb(:,i) = Quaternion2Euler(qbb(:,i))/pi*180;
%     Cbb(:,:,i) = Quaternion2ACM(qbb(:,i));
end
qbb2 = qbb(:,1:5:D_N);
D_GPS_ODO_V = D_GPS_V(:,1:2:D_N_GPS);
D_ODO_IMU_N = size(qbb2,2);
D_GPS_ODO_N = size(D_GPS_ODO_V,2);

for i = 1:size(qbb2,2)
    Cbb(:,:,i) = Quaternion2ACM(qbb2(:,i));
    v(:,i) = Cbb(:,:,i)*[ODO_vel(i,3);ODO_vel(i,2);0];
end
vb = [v(2,:);v(1,:);v(3,:)];
for i = 2:size(qbb2,2)
%     XYZ(:,i) = LocationUpdata(XYZ(:,i-1),v(:,i),v(:,i-1),D_T);
    XYZ(:,i) =  LocationUpdata3(XYZ(:,i-1),vb(:,i),vb(:,i-1),1.2,WGS84);
end


figure(1);
plot3(XYZ(1,:),XYZ(2,:),zeros(size(XYZ,2),1),'*');
% figure(2);
hold on;
plot3(RTK_lla(:,2),RTK_lla(:,3),zeros(size(RTK_lla,1),1));

figure(4);subplot(2,1,1);hold on;
plot([1:D_N_GPS]*0.01,D_GPS_V(2,:));
hold on;plot([1:D_ODO_IMU_N]*0.02,vb(1,:));
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
title('ODOǰ���ٶ�ת����ϵ/GNSS�����ٶ�');
legend('GPS','ODO');
subplot(2,1,2);hold on;
plot([1:D_N_GPS]*0.01,D_GPS_V(1,:));
hold on;plot([1:D_ODO_IMU_N]*0.02,vb(2,:));
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
title('ODO�����ٶ�ת����ϵ/GNSS�����ٶ�');
legend('GPS','ODO');

figure(10);subplot(3,1,1);hold on;
plot([1:D_N_GPS]*0.01,RTK_att(end,:));
hold on;plot([1:D_N]*0.004,Eulerbb(3,:));
ylabel('ƫ����(��)');xlabel('ʱ��(s)');
legend('IPMV','IMU');title('���˳�ƫ����');
subplot(3,1,2);hold on;
plot([1:D_N_GPS]*0.01,RTK_att(1,:));
hold on;plot([1:D_N]*0.004,Eulerbb(1,:));
ylabel('������(��)');xlabel('ʱ��(s)');
legend('IPMV','IMU');title('���˳�������');
subplot(3,1,3);hold on;
plot([1:D_N_GPS]*0.01,RTK_att(2,:));
hold on;plot([1:D_N]*0.004,Eulerbb(2,:));
ylabel('�����(��)');xlabel('ʱ��(s)');
legend('IPMV','IMU');title('���˳������');