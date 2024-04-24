clc;
clear;
close all;
format long
% 娴嬭瘯濮挎?佹洿鏂板嚱鏁?(AttitudeUpdate_single_sample.m)

% 鍖呭惈璺緞,鑻ユ浠ｇ爜瀹炴晥锛屾墜鍔ㄦ坊鍔?
addpath('./data');
addpath('./INS_lib');
addpath('./Kalman_f');
load("err_ekf.mat");
load("DATA.mat")
%                                                                                m/s     銆?        rad/s     s
load("alldata_800_45_1_0005_0.mat");              %鐞嗚寮归亾,鍛藉悕鏍煎紡,    alldata   鍑鸿啗鍒濋??  灏勮(淇话)  鍑鸿啗杞??  閲囨牱鏃堕棿
A_Data = load("TEST_800_45_1_0005_0.mat");        %妯℃嫙浼犳劅鍣?,鍛藉悕鏍煎紡,     TEST    鍑鸿啗鍒濋??  灏勮(淇话)  鍑鸿啗杞??  閲囨牱鏃堕棿 
B_SensorData = A_Data.Z_IMU;
D_N = size(B_SensorData,1);
D_T = 0.005;

%                 XYZ
% Z_IMU鍧愭爣绯讳负涓滃寳澶xyz
% D_ACC = [B_SensorData(:,4),B_SensorData(:,5),B_SensorData(:,6)]';          %鎸夌収绠楁硶椤哄簭閲嶆帓鍔犻?熷害涓夎酱璇绘暟椤哄簭 x ,  y  , z
% D_GYR = [B_SensorData(:,1),B_SensorData(:,2),B_SensorData(:,3)]';          %鎸夌収绠楁硶椤哄簭閲嶆帓闄?铻轰华涓夎酱璇绘暟椤哄簭pitch,row,yaw                                                                         %                             x    y    z


% 鐢熸垚鍣０鍜岄殢鏈烘父璧?
r_walk = random_walk(D_T,D_N*D_T,D_N,0.0001);
w_noise = mvnrnd(0,0.0005,D_N);
rw = r_walk(1:D_N,1) + w_noise;
rw = w_noise;
% w_noise = 0;
% rw = 0;
% Z_IMU浣跨敤鐨勬儻瀵兼牸寮?

% 娣诲姞鍣０鍜岄殢鏈烘父璧?
D_ACC = [B_SensorData(:,4)+rw,B_SensorData(:,5)+rw,B_SensorData(:,6)+rw]';
D_GYR = [B_SensorData(:,1)+rw,B_SensorData(:,2)+rw,B_SensorData(:,3)+rw]';
GPS_Y = [Y(:,3)+w_noise,Y(:,1)+w_noise,Y(:,2)+w_noise];

Ballistic_data = [Y(:,3),Y(:,1),Y(:,2)];                                   %妯℃嫙寮归亾
GPS_test = Generate_GPS(GPS_Y,100,200,D_N);                                %100Hz   GPS
GPS_interpolate = Interpolation_GPS(GPS_Y,Ballistic_data,100,200,D_N);     %

% 娣诲姞鍣０鍜岄殢鏈烘父璧?
D_ACC = [Data(:,4),Data(:,5),Data(:,6)]';
D_GYR = [Data(:,1),Data(:,2),Data(:,3)]';

%%%%%%%%%%%%%%%%%%%%%%%%%%鍦扮悆妯″瀷(WGS84绠?鍖栧弬鏁?)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WGS84.a = 6371100;                                                         %闀垮崐杞?
WGS84.b = 6371100;                                                         %鐭崐杞?
WGS84.R = 6371100;                                                         %骞冲潎鍗婂緞
WGS84.f = 0;                                                               %鎵佺巼
WGS84.e = 0;                                                               %鍋忓績鐜?
WGS84.e2 =0;                                                               %绗竴鍋忓績鐜囧钩鏂?
WGS84.ep2=0;                                                               %绗簩鍋忓績鐜囧钩鏂?
WGS84.we =7.292115e-5;                                                     %鍦扮悆鑷浆瑙掗?熺巼
WGS84.GM =3.986004418e+14;                                                 %鍦扮悆寮曞姏涓哄父鏁?
% WGS84.ge=9.7803267715;                                                   %璧ら亾閲嶅姏鍔犻?熷害
WGS84.ge =9.80;                                                            %甯歌閲嶅姏鍔犻?熷害
WGS84.gp =9.8321863685;                                                    %鏋佸湴閲嶅姏鍔犻?熷害
%%%%%%%%%%%%%%%%%%%%%%%%%%鍦扮悆妯″瀷(WGS84绠?鍖栧弬鏁?)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%浣嶅Э鍒濆鍖?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
XYZ(:,1) = [0;0;0];                                                        %浣嶇疆
BLH(:,1) = [0;0;0];                                                        %绾粡楂?
v(:,1) = [Data(1,7);Data(1,8);Data(1,9)];                                  %閫熷害
Delta_sita = [0;0;0];                                                      %瑙掑閲?
% Euler(:,1) = [45/180*pi;0/180*pi;0/180*pi];                                              %鍒濆娆ф媺瑙抪itch,yaw,row(寮у害鍒?)
Euler(:,1) = [40/180*pi;145/180*pi;0/180*pi];                                              %鍒濆娆ф媺瑙抪itch,yaw,row(寮у害鍒?)
qbn(:,1) = Euler2Quaternion(Euler(1),Euler(2),Euler(3));                   %鍒濆濮挎?佸洓鍏冩暟,涓滃寳澶╁潗鏍囩郴锛屾棆杞『搴?(zxy)312
Cbn(:,:,1) = Quaternion2ACM(qbn(:,1));                                     %鍒濆濮挎?佹棆杞煩闃?,涓滃寳澶╁潗鏍囩郴锛屾棆杞『搴?(zxy)312
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%浣嶅Э鍒濆鍖?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%kalman鍒濆鍖?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%kalman鍒濆鍖?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 2:D_N                                          
%--------------------------------1.濮挎?佺煩闃佃В绠?------------------------------
    Delta_sita(:,i) = D_GYR(:,i-1)*D_T+(D_GYR(:,i)-D_GYR(:,i-1))/2*D_T;
    
    qbn(:,i) = AttitudeUpdate_single_sample(qbn(:,i-1), Delta_sita(:,i-1),Delta_sita(:,i));

    Cbn(:,:,i) = Quaternion2ACM(qbn(:,i));
    Euler(:,i) = Quaternion2Euler(qbn(:,i));
%--------------------------------2.閫熷害鏇存柊---------------------------------
    v(:,i) = VelocityUpdate3(v(:,i-1),D_T,Cbn(:,:,i),Cbn(:,:,i-1),D_ACC(:,i),D_ACC(:,i-1));
%--------------------------------3.浣嶇疆鏇存柊---------------------------------
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
ylabel('y轴飞行距离(m)');xlabel('x轴飞行距离(m)');zlabel('z轴飞行距离(m)');
title('惯性导肮结果');
legend('IMU解算轨迹','实际弹道');



% figure(2);
% subplot(3,1,1);hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,1),'k');hold on;grid on;
% ylabel('x轴误差(m)');xlabel('时间(s)');
% title('x轴惯性导肮误差');legend('位置误差');
% subplot(3,1,2);hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,2),'k');hold on;grid on;
% ylabel('y轴误差(m)');xlabel('时间(s)');
% title('y轴惯性导肮误差');legend('位置误差');
% subplot(3,1,3);hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,3),'k');hold on;grid on;
% ylabel('z轴误差(m)');xlabel('时间(s)');
% title('z轴惯性导肮误差');legend('位置误差');
% 
% figure(9);clf(9)
% subplot(3,1,1);hold on;
% plot([1:10:D_N]*0.005,Z_Err2(:,1),'r');legend('GPS位置误差');hold on;
% plot([1:10:D_N]*0.005,Z_Err(:,1),'k');legend('组合导航位置误差');hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,1),'b');legend('IMU位置误差');hold on;grid on;
% ylabel('x轴向位置误差(m)');xlabel('时间(s)');
% title("x轴向位置误差");grid on;
% subplot(3,1,2);hold on;
% plot([1:10:D_N]*0.005,Z_Err2(:,1),'r');legend('GPS位置误差');hold on;
% plot([1:10:D_N]*0.005,Z_Err(:,1),'k');legend('组合导航位置误差');hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,1),'b');legend('IMU位置误差');hold on;grid on;
% ylabel('y轴向位置误差(m)');xlabel('时间(s)');
% title("y轴向位置误差");grid on;
% subplot(3,1,3);hold on;
% plot([1:10:D_N]*0.005,Z_Err2(:,1),'r');legend('GPS位置误差');hold on;
% plot([1:10:D_N]*0.005,Z_Err(:,1),'k');legend('组合导航位置误差');hold on;
% plot([1:10:D_N]*0.005,Z_Err3(:,1),'b');legend('IMU位置误差');hold on;grid on;
% ylabel('z轴向位置误差(m)');xlabel('时间(s)');
% title("z轴向位置误差");grid on;
% 
% figure(7);clf(7);
% subplot(3,1,1);hold on;
% plot([1:D_N]*0.005,Euler(1,:),'k');legend('角度');
% ylabel('俯仰角(°)');xlabel('时间(s)');title("俯仰角曲线");grid on;
% subplot(3,1,2);hold on;
% plot([1:1:D_N]*0.005,Euler(2,:),'k');legend('角度');
% ylabel('横滚角(°)');xlabel('时间(s)');title("横滚角曲线");grid on;
% subplot(3,1,3);hold on;
% plot([1:1:D_N]*0.005,Euler(3,:)+60,'k');legend('角度');
% ylabel('偏航角(°)');xlabel('时间(s)');title("偏航角曲线");grid on;
% 
% figure(8);clf(8);
% plot(w_noise,'k');
% title("高斯噪声");grid on;