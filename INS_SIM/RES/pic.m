clc;clear;close all;
load("res_17885.mat");
% load("res_1772.mat");
% D_N = 40000;

figure(1);
subplot(3,1,1);hold on;
plot([1:D_N]*0.004,Eulerbb(end,1:D_N));hold on;
ylabel('ƫ����(��)');xlabel('ʱ��(s)');
title('INS����(ƫ��)');legend('IMU');
subplot(3,1,2);hold on;
plot([1:D_N]*0.004,Eulerbb(1,1:D_N));hold on;
ylabel('������(��)');xlabel('ʱ��(s)');
title('INS����(����)');legend('IMU');
subplot(3,1,3);hold on;
plot([1:D_N]*0.004,Eulerbb(2,1:D_N));hold on;
ylabel('�����(��)');xlabel('ʱ��(s)');
title('INS����(���)');legend('IMU');

figure(4);plot([1:D_N_GPS]*0.01,D_GPS_V(2,:));
hold on;plot([1:D_ODO_IMU_N]*0.02,v(2,:));
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
title('ODOǰ���ٶ�ת����ϵ/GNSS�����ٶ�');
legend('GPS','ODO');

figure(5);plot([1:D_N_GPS]*0.01,D_GPS_V(1,:));
hold on;plot([1:D_ODO_IMU_N]*0.02,v(1,:));
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
title('ODO�����ٶ�ת����ϵ/GNSS�����ٶ�');
legend('GPS','ODO');

figure(6);
% plot(BAS.x_store(4,:));
plot([1:size(Euler,2)]*0.085,Euler(3,1:size(Euler,2)));
hold on;plot([1:1000-300]*0.125,BAS.xbest_store(1:1000-300,3));
ylabel('ƫ����(��)');xlabel('ʱ��(s)');
hold on;legend('SVD','BAS');
title('��ʼƫ����̬���ƽ��');

figure(7);
plot([1:size(Euler,2)-100]*0.05,Euler(1,1:size(Euler,2)-100));
hold on;plot([1:1000-100]*0.05,BAS.xbest_store(1:1000-100,1));
ylabel('������(��)');xlabel('ʱ��(s)');
hold on;legend('SVD','BAS');
title('��ʼ������̬���ƽ��');

figure(8);
plot([1:size(Euler,2)-100]*0.05,Euler(2,1:size(Euler,2)-100));
hold on;plot([1:1000-100]*0.05,BAS.xbest_store(1:1000-100,2));
ylabel('�����(��)');xlabel('ʱ��(s)');
hold on;legend('SVD','BAS');
title('��ʼ�����̬���ƽ��');

figure(9);
plot3(RTK_lla(1:end,2),RTK_lla(1:end,3),RTK_lla(1:end,4));
hold on;
plot3(RTK_lla(1,2),RTK_lla(1,3),RTK_lla(1,4),'*');
hold on;
plot3(RTK_lla(end,2),RTK_lla(end,3),RTK_lla(end,4),'O');
hold on;legend('�˶��켣','���','�յ�');
ylabel('����(��)');xlabel('γ��(��)');
title('���˳��˶��켣');

figure(10);subplot(3,1,1);hold on;
plot([1:D_N_GPS]*0.01,RTK_att(end,:));
ylabel('ƫ����(��)');xlabel('ʱ��(s)');
legend('IPMV');title('���˳�ƫ����');
subplot(3,1,2);hold on;
plot([1:D_N_GPS]*0.01,RTK_att(1,:));
ylabel('������(��)');xlabel('ʱ��(s)');
legend('IPMV');title('���˳�������');
subplot(3,1,3);hold on;
plot([1:D_N_GPS]*0.01,RTK_att(2,:));
ylabel('�����(��)');xlabel('ʱ��(s)');
legend('IPMV');title('���˳������');

figure(11);subplot(3,1,1);hold on;
plot([1:D_N_GPS]*0.01,D_GPS_V(end,:));
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
legend('IPMV');title('���˳������ٶ�');
subplot(3,1,2);hold on;
plot([1:D_N_GPS]*0.01,D_GPS_V(1,:));
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
legend('IPMV');title('���˳������ٶ�');
subplot(3,1,3);hold on;
plot([1:D_N_GPS]*0.01,D_GPS_V(2,:));
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
legend('IPMV');title('���˳������ٶ�');

figure(12);hold on;
plot([1:size(ODO_vel,1)]*0.02,ODO_vel(:,2))
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
legend('ODO');title('ODO����ٶ�');

figure(13);
subplot(2,1,1);hold on;
hold on;plot([1:D_ODO_IMU_N]*0.02,v(2,:));
% hold on;plot([1:D_N_GPS]*0.01,D_GPS_V(2,:));
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
title('ODO/IMUǰ���ٶ�');legend('�ٶ�');
subplot(2,1,2);hold on;
hold on;plot([1:D_ODO_IMU_N]*0.02,v(1,:));
% hold on;plot([1:D_N_GPS]*0.01,D_GPS_V(1,:));
ylabel('�ٶ�(m/s)');xlabel('ʱ��(s)');
title('ODO/IMU�����ٶ�');legend('�ٶ�');

BASpit_err = RTK_att(1,1)-BAS.xbest_store(1000,1)
BASrol_err = RTK_att(2,1)-BAS.xbest_store(1000,2)
BASyaw_err = RTK_att(3,1)-BAS.xbest_store(1000,3)

SVDpit_err = RTK_att(1,1)-Euler(1,end)
SVDrol_err = RTK_att(2,1)-Euler(2,end)
SVDyaw_err = RTK_att(3,1)-Euler(3,end)