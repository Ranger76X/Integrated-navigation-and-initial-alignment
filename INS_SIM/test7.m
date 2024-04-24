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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%地球物理量(wgs84)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WGS84.a = 6371100;                                                        
WGS84.b = 6371100;                                                         
WGS84.R = 6371100;                                                         
WGS84.f = 0;                                                               
WGS84.e = 0;                                                               
WGS84.e2 =0;                                                               
WGS84.ep2=0;                                                               
WGS84.we =7.292115e-5;                                                     
WGS84.GM =3.986004418e+14;                                                                                                  
WGS84.ge =9.80;                                                            
WGS84.gp =9.8321863685;                                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%地球物理量(wgs84)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eulerbb(:,1) = [RTK_att(1,1)/180*pi;RTK_att(2,1)/180*pi;RTK_att(3,1)/180*pi];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%导航初始参数%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eulerbb(:,1) = [0/180*pi;0/180*pi;-175/180*pi];
Eulerbb(:,1) = [0/180*pi;0/180*pi;0/180*pi];
qbb(:,1) = Euler2Quaternion(Eulerbb(1),Eulerbb(2),Eulerbb(3));                   
Cbb(:,:,1) = Quaternion2ACM(qbb(:,1));                                                                                                            
v(:,1) = [0;0;0];                                           
Delta_sita = [0;0;0];                                                                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%导航初始参数%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 2:D_N                                         
%--------------------------------1.姿态更新------------------------------
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
    v(:,i) = Cbb(:,:,i)*[0;ODO_vel(i,2);0];
end

A = 0;
for i = 9287:9287+5000
    A = A + D_GPS_ODO_V(:,i) * v(:,i)';
    % svd
    [U,D,V] = svds(A);
    % best Att Matrix
    C = U*V';
    Euler(:,i) = m2att(C)*180/pi;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%鏁版嵁鏄剧ず閮ㄥ垎
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 1000;
start = 9287;
BAS.eta = 0.9;
BAS.c = 1;                  %ratio between step and d0
BAS.step = 10;               %initial step set as the largest input range
BAS.n = 200;                %iterations
BAS.k = 3;                  %space dimension
BAS.x = [1;1;0];        %intial value
BAS.xbest = BAS.x;
BAS.f = f3_2(BAS.x,N,v,D_GPS_ODO_V,start);
BAS.fbest = BAS.f;
BAS.fbest_store = BAS.fbest;
BAS.x_store = [0;BAS.x;BAS.fbest];
for i = 1:BAS.n
    BAS.d0=BAS.step/BAS.c;
    BAS.dir=rands(BAS.k,1);
    BAS.dir=BAS.dir/(eps+norm(BAS.dir));    
    
    if BAS.x(3) > 180
        BAS.x(3) = BAS.x(3) - 360;
    end
    
    BAS.xleft=BAS.x+BAS.dir*BAS.d0;
    BAS.fleft=f3_2(BAS.xleft,N,v,D_GPS_ODO_V,start);
    BAS.xright=BAS.x-BAS.dir*BAS.d0;
    BAS.fright=f3_2(BAS.xright,N,v,D_GPS_ODO_V,start);    
    
    BAS.x=BAS.x-BAS.step*BAS.dir*sign(BAS.fleft-BAS.fright);
    
    BAS.f = f3_2(BAS.x,N,v,D_GPS_ODO_V,start);
    
    %%%%%%%%%%
    if BAS.f<BAS.fbest
        BAS.xbest=[mod(BAS.x(1),90);mod(BAS.x(2),90);mod(BAS.x(3),180);];
        BAS.fbest=BAS.f;        
    end
    %%%%%%%%%%%
    BAS.x_store=cat(2,BAS.x_store,[i;BAS.x;BAS.f]);
    BAS.fbest_store=[BAS.fbest_store;BAS.fbest];
    BAS.xbest_store(i,:) = BAS.xbest; 
    display([num2str(i),':xbest=[',num2str(BAS.xbest'),'],fbest=',num2str(BAS.fbest)])
    %%%%%%%%%%%
    if(BAS.fbest >= 0.2)
        BAS.step=BAS.step;
    else
        BAS.step=BAS.step*BAS.eta;
    end
end
pitch = 1;
yaw = BAS.x(3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%鏁版嵁鏄剧ず閮ㄥ垎
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BAS.eta = 0.9;
BAS.c = 1;                  %ratio between step and d0
BAS.step = 10;               %initial step set as the largest input range
BAS.n = 1000;                %iterations
BAS.k = 3;                  %space dimension
BAS.x = [pitch;1;yaw];        %intial value
BAS.xbest = BAS.x;
BAS.f = f3_2(BAS.x,N,v,D_GPS_ODO_V,start);
BAS.fbest = BAS.f;
BAS.fbest_store = BAS.fbest;
BAS.x_store = [0;BAS.x;BAS.fbest];


for i = 1:BAS.n
    BAS.d0=BAS.step/BAS.c;
    BAS.dir=rands(BAS.k,1);
    BAS.dir=BAS.dir/(eps+norm(BAS.dir));    
    
    if BAS.x(3) > 180
        BAS.x(3) = BAS.x(3) - 360;
    end
    
    BAS.xleft=BAS.x+BAS.dir*BAS.d0;
    BAS.fleft=f3_2(BAS.xleft,N,v,D_GPS_ODO_V,start);
    BAS.xright=BAS.x-BAS.dir*BAS.d0;
    BAS.fright=f3_2(BAS.xright,N,v,D_GPS_ODO_V,start);    
    
    BAS.x=BAS.x-BAS.step*BAS.dir*sign(BAS.fleft-BAS.fright);
    
    BAS.f = f3_2(BAS.x,N,v,D_GPS_ODO_V,start);
    
    %%%%%%%%%%
    if BAS.f<BAS.fbest
        BAS.xbest=BAS.x;
        BAS.fbest=BAS.f;        
    end
    %%%%%%%%%%%
    BAS.x_store=cat(2,BAS.x_store,[i;BAS.x;BAS.f]);
    BAS.fbest_store=[BAS.fbest_store;BAS.fbest];
    BAS.xbest_store(i,:) = BAS.xbest; 
    display([num2str(i),':xbest=[',num2str(BAS.xbest'),'],fbest=',num2str(BAS.fbest)])
    %%%%%%%%%%%
    if(BAS.fbest >= 0.2)
        BAS.step=BAS.step;
    else
        BAS.step=BAS.step*BAS.eta;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%鏁版嵁鏄剧ず閮ㄥ垎
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display([num2str(i),':real=[',num2str([1;1;-180]'),'],fbest=',num2str(f3_2([1,1,-180],N,v,D_GPS_ODO_V,start))])

figure(1);plot([1:D_N_GPS]*0.01,RTK_att(end,:));
hold on;plot([1:D_N]*0.004,Eulerbb(end,:));
title('INS惯性推算/GNSS绝对观测(偏航)');
legend('GPS','IMU');

figure(2);plot([1:D_N_GPS]*0.01,RTK_att(1,:));
hold on;plot([1:D_N]*0.004,Eulerbb(1,:));
title('INS惯性推算/GNSS绝对观测(俯仰)');
legend('GPS','IMU');

figure(4);plot([1:D_N]*0.004,Eulerbb(2,:));
hold on;plot([1:D_N_GPS]*0.01,RTK_att(2,:));
title('INS惯性推算/GNSS绝对观测(横滚)');
legend('GPS','IMU');

figure(5);plot([1:D_N_GPS]*0.01,D_GPS_V(2,:));
hold on;plot([1:D_ODO_IMU_N]*0.02,v(2,:));
title('ODO前向速度转导航系/GNSS北向速度');
legend('GPS','ODO');

figure(6);plot([1:D_N_GPS]*0.01,D_GPS_V(1,:));
hold on;plot([1:D_ODO_IMU_N]*0.02,v(1,:));
title('ODO右向速度转导航系/GNSS东向速度');
legend('GPS','ODO');

figure(7);plot(BAS.x_store(4,:));
hold on;plot(BAS.xbest_store(:,3));
hold on;legend('BAS','BASR');
title('初始偏航姿态对准结果');
Euler(:,end)
