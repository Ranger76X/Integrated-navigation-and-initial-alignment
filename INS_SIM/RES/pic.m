clc;clear;close all;
load("res_17885.mat");
% load("res_1772.mat");
% D_N = 40000;

figure(1);
subplot(3,1,1);hold on;
plot([1:D_N]*0.004,Eulerbb(end,1:D_N));hold on;
ylabel('偏航角(°)');xlabel('时间(s)');
title('INS推算(偏航)');legend('IMU');
subplot(3,1,2);hold on;
plot([1:D_N]*0.004,Eulerbb(1,1:D_N));hold on;
ylabel('俯仰角(°)');xlabel('时间(s)');
title('INS推算(俯仰)');legend('IMU');
subplot(3,1,3);hold on;
plot([1:D_N]*0.004,Eulerbb(2,1:D_N));hold on;
ylabel('横滚角(°)');xlabel('时间(s)');
title('INS推算(横滚)');legend('IMU');

figure(4);plot([1:D_N_GPS]*0.01,D_GPS_V(2,:));
hold on;plot([1:D_ODO_IMU_N]*0.02,v(2,:));
ylabel('速度(m/s)');xlabel('时间(s)');
title('ODO前向速度转导航系/GNSS北向速度');
legend('GPS','ODO');

figure(5);plot([1:D_N_GPS]*0.01,D_GPS_V(1,:));
hold on;plot([1:D_ODO_IMU_N]*0.02,v(1,:));
ylabel('速度(m/s)');xlabel('时间(s)');
title('ODO右向速度转导航系/GNSS东向速度');
legend('GPS','ODO');

figure(6);
% plot(BAS.x_store(4,:));
plot([1:size(Euler,2)]*0.085,Euler(3,1:size(Euler,2)));
hold on;plot([1:1000-300]*0.125,BAS.xbest_store(1:1000-300,3));
ylabel('偏航角(°)');xlabel('时间(s)');
hold on;legend('SVD','BAS');
title('初始偏航姿态估计结果');

figure(7);
plot([1:size(Euler,2)-100]*0.05,Euler(1,1:size(Euler,2)-100));
hold on;plot([1:1000-100]*0.05,BAS.xbest_store(1:1000-100,1));
ylabel('俯仰角(°)');xlabel('时间(s)');
hold on;legend('SVD','BAS');
title('初始俯仰姿态估计结果');

figure(8);
plot([1:size(Euler,2)-100]*0.05,Euler(2,1:size(Euler,2)-100));
hold on;plot([1:1000-100]*0.05,BAS.xbest_store(1:1000-100,2));
ylabel('横滚角(°)');xlabel('时间(s)');
hold on;legend('SVD','BAS');
title('初始横滚姿态估计结果');

figure(9);
plot3(RTK_lla(1:end,2),RTK_lla(1:end,3),RTK_lla(1:end,4));
hold on;
plot3(RTK_lla(1,2),RTK_lla(1,3),RTK_lla(1,4),'*');
hold on;
plot3(RTK_lla(end,2),RTK_lla(end,3),RTK_lla(end,4),'O');
hold on;legend('运动轨迹','起点','终点');
ylabel('经度(°)');xlabel('纬度(°)');
title('无人车运动轨迹');

figure(10);subplot(3,1,1);hold on;
plot([1:D_N_GPS]*0.01,RTK_att(end,:));
ylabel('偏航角(°)');xlabel('时间(s)');
legend('IPMV');title('无人车偏航角');
subplot(3,1,2);hold on;
plot([1:D_N_GPS]*0.01,RTK_att(1,:));
ylabel('俯仰角(°)');xlabel('时间(s)');
legend('IPMV');title('无人车俯仰角');
subplot(3,1,3);hold on;
plot([1:D_N_GPS]*0.01,RTK_att(2,:));
ylabel('横滚角(°)');xlabel('时间(s)');
legend('IPMV');title('无人车横滚角');

figure(11);subplot(3,1,1);hold on;
plot([1:D_N_GPS]*0.01,D_GPS_V(end,:));
ylabel('速度(m/s)');xlabel('时间(s)');
legend('IPMV');title('无人车天向速度');
subplot(3,1,2);hold on;
plot([1:D_N_GPS]*0.01,D_GPS_V(1,:));
ylabel('速度(m/s)');xlabel('时间(s)');
legend('IPMV');title('无人车北向速度');
subplot(3,1,3);hold on;
plot([1:D_N_GPS]*0.01,D_GPS_V(2,:));
ylabel('速度(m/s)');xlabel('时间(s)');
legend('IPMV');title('无人车东向速度');

figure(12);hold on;
plot([1:size(ODO_vel,1)]*0.02,ODO_vel(:,2))
ylabel('速度(m/s)');xlabel('时间(s)');
legend('ODO');title('ODO测得速度');

figure(13);
subplot(2,1,1);hold on;
hold on;plot([1:D_ODO_IMU_N]*0.02,v(2,:));
% hold on;plot([1:D_N_GPS]*0.01,D_GPS_V(2,:));
ylabel('速度(m/s)');xlabel('时间(s)');
title('ODO/IMU前向速度');legend('速度');
subplot(2,1,2);hold on;
hold on;plot([1:D_ODO_IMU_N]*0.02,v(1,:));
% hold on;plot([1:D_N_GPS]*0.01,D_GPS_V(1,:));
ylabel('速度(m/s)');xlabel('时间(s)');
title('ODO/IMU右向速度');legend('速度');

BASpit_err = RTK_att(1,1)-BAS.xbest_store(1000,1)
BASrol_err = RTK_att(2,1)-BAS.xbest_store(1000,2)
BASyaw_err = RTK_att(3,1)-BAS.xbest_store(1000,3)

SVDpit_err = RTK_att(1,1)-Euler(1,end)
SVDrol_err = RTK_att(2,1)-Euler(2,end)
SVDyaw_err = RTK_att(3,1)-Euler(3,end)