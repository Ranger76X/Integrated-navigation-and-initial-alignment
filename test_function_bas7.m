%100´ÎÊµÑé·ÂÕæ

clc;
clear;
close all;
format long
% æµ‹è¯•å§¿æ?æ›´æ–°å‡½æ•?(AttitudeUpdate_single_sample.m)

% åŒ…å«è·¯å¾„,è‹¥æ­¤ä»£ç å®æ•ˆï¼Œæ‰‹åŠ¨æ·»åŠ?
addpath('./data');
addpath('./INS_lib');
addpath('./Kalman_f');
addpath('./Att_init_lib');
%                                                                                m/s     ã€?        rad/s     s
load("alldata_800_45_1_0005_0_75_45_30.mat");              %ç†è®ºå¼¹é“,å‘½åæ ¼å¼,    alldata   å‡ºè†›åˆé??  å°„è§’(ä¿¯ä»°)  å‡ºè†›è½¬é??  é‡‡æ ·æ—¶é—´
A_Data = load("TEST_800_45_1_0005_0_75_45_30.mat");        %æ¨¡æ‹Ÿä¼ æ„Ÿå™?,å‘½åæ ¼å¼,     TEST    å‡ºè†›åˆé??  å°„è§’(ä¿¯ä»°)  å‡ºè†›è½¬é??  é‡‡æ ·æ—¶é—´ 
B_SensorData = A_Data.Z_IMU;
D_N = size(B_SensorData,1);
D_T = 0.005;

% ç”Ÿæˆå™ªå£°å’Œéšæœºæ¸¸èµ?
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
plot3(GPS_Y(:,1),GPS_Y(:,2),GPS_Y(:,3),'K.');                              %ç»˜åˆ¶å¼¹é“
%%%%%%%%%%%%%%%%%%%%%%%%%%åœ°çƒæ¨¡å‹(WGS84ç®?åŒ–å‚æ•?)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WGS84.a = 6371100;                                                         %é•¿åŠè½?
WGS84.b = 6371100;                                                         %çŸ­åŠè½?
WGS84.R = 6371100;                                                         %å¹³å‡åŠå¾„
WGS84.f = 0;                                                               %æ‰ç‡
WGS84.e = 0;                                                               %åå¿ƒç?
WGS84.e2 =0;                                                               %ç¬¬ä¸€åå¿ƒç‡å¹³æ–?
WGS84.ep2=0;                                                               %ç¬¬äºŒåå¿ƒç‡å¹³æ–?
WGS84.we =7.292115e-5;                                                     %åœ°çƒè‡ªè½¬è§’é?Ÿç‡
WGS84.GM =3.986004418e+14;                                                 %åœ°çƒå¼•åŠ›ä¸ºå¸¸æ•?
% WGS84.ge=9.7803267715;                                                   %èµ¤é“é‡åŠ›åŠ é?Ÿåº¦
WGS84.ge =9.80;                                                            %å¸¸è§„é‡åŠ›åŠ é?Ÿåº¦
WGS84.gp =9.8321863685;                                                    %æåœ°é‡åŠ›åŠ é?Ÿåº¦
%%%%%%%%%%%%%%%%%%%%%%%%%%åœ°çƒæ¨¡å‹(WGS84ç®?åŒ–å‚æ•?)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

qbb(:,1) = [1;0;0;0];
Cbb(:,:,1) = Quaternion2ACM(qbb(:,1));
Eulerbb(:,1) = [0;0;0];

Cbn(:,:,1) = eye(3);
qbn(:,1) = [1;0;0;0];
Euler(:,1) = [0;0;0];                                                      %åˆå§‹æ¬§æ‹‰è§’pitch,yaw,row(å¼§åº¦åˆ?)

vb(:,1) = [0;0;0];
vn = [GPS_V(:,1),GPS_V(:,2),GPS_V(:,3)]';
Beta(:,1) = [0;0;0];
Delta_sita(:,1) = [0;0;0];
A = 0;
for i = 2:D_N                                          
%--------------------------------1.å§¿æ?çŸ©é˜µè§£ç®?------------------------------
    Delta_sita(:,i) = D_GYR(:,i-1)*D_T+(D_GYR(:,i)-D_GYR(:,i-1))/2*D_T;   
    qbb(:,i) = AttitudeUpdate_single_sample(qbb(:,i-1), Delta_sita(:,i-1),Delta_sita(:,i));
    Eulerbb(:,i) = Quaternion2Euler(qbb(:,i))/pi*180;
    Cbb(:,:,i) = Quaternion2ACM(qbb(:,i));
%--------------------------------2.é€Ÿåº¦æ›´æ–°---------------------------------
    vb(:,i) = VelocityUpdate_SVD(vb(:,i-1),D_T,Cbb(:,:,i),Cbb(:,:,i-1),D_ACC(:,i),D_ACC(:,i-1));
%--------------------------------3.ä½ç½®æ›´æ–°---------------------------------  
    Beta(:,i) = vn(:,i) - vn(:,1);   
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%BASåˆå§‹åŒ–éƒ¨åˆ?1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll = 0;
% roll = 75.5;

% roll = atan(D_GYR(3,1)/D_GYR(1,1))/pi*180;
for j = 1:100
    roll = atan(D_GYR(3,1)/D_GYR(1,1))/pi*180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%BASåˆå§‹åŒ–éƒ¨åˆ?1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    N = 60;%éœ?è¦çš„è§‚æµ‹çŸ¢é‡å¯¹æ•°
    BAS.eta = 0.9;
    BAS.c = 1;                  %ratio between step and d0
    BAS.step = 1;               %initial step set as the largest input range
    BAS.n = 500;                %iterations
    BAS.k = 3;                  %space dimension
    BAS.x = [0;roll;0];        %intial value
    BAS.xbest = BAS.x;
    BAS.f = f3_2(BAS.x,N,vb,Beta);
    BAS.fbest = BAS.f;
    BAS.fbest_store = BAS.fbest;
    BAS.x_store = [0;BAS.x;BAS.fbest];


    for i = 1:BAS.n
        BAS.d0=BAS.step/BAS.c;
        BAS.dir=rands(BAS.k,1);
        BAS.dir=BAS.dir/(eps+norm(BAS.dir));    

        BAS.xleft=BAS.x+BAS.dir*BAS.d0;
        BAS.fleft=f3_2(BAS.xleft,N,vb,Beta);
        BAS.xright=BAS.x-BAS.dir*BAS.d0;
        BAS.fright=f3_2(BAS.xright,N,vb,Beta);    

        BAS.x=BAS.x-BAS.step*BAS.dir*sign(BAS.fleft-BAS.fright);

        BAS.f = f3_2(BAS.x,N,vb,Beta);

        %%%%%%%%%%
        if BAS.f<BAS.fbest
            BAS.xbest=BAS.x;
            BAS.fbest=BAS.f;        
        end
        %%%%%%%%%%%
        BAS.x_store=cat(2,BAS.x_store,[i;BAS.x;BAS.f]);
        BAS.fbest_store=[BAS.fbest_store;BAS.fbest];
        BAS.xbest_store(i,:) = BAS.xbest; 
        BAS.x_store2(:,i) = [i;BAS.x;BAS.f];
        %%%%%%%%%%%
        if(BAS.fbest >= 0.4)
            BAS.step=BAS.step;
        else
            BAS.step=BAS.step*BAS.eta;
        end
    end
    pitch = BAS.x(1);
    yaw = BAS.x(3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %æ•°æ®æ˜¾ç¤ºéƒ¨åˆ†
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    N = 60;%éœ?è¦çš„è§‚æµ‹çŸ¢é‡å¯¹æ•°
    BAS.eta = 0.6;
    BAS.c = 1;                  %ratio between step and d0
    BAS.step = 1;               %initial step set as the largest input range
    BAS.n = 500;                %iterations
    BAS.k = 3;                  %space dimension
    BAS.x = [pitch;roll;yaw];        %intial value
    BAS.xbest = BAS.x;
    BAS.f = f3_2(BAS.x,N,vb,Beta);
    BAS.fbest = BAS.f;
    BAS.fbest_store = BAS.fbest;
    BAS.x_store = [0;BAS.x;BAS.fbest];


    for i = 500:500+BAS.n
        BAS.d0=BAS.step/BAS.c;
        BAS.dir=rands(BAS.k,1);
        BAS.dir=BAS.dir/(eps+norm(BAS.dir));    

        BAS.xleft=BAS.x+BAS.dir*BAS.d0;
        BAS.fleft=f3_2(BAS.xleft,N,vb,Beta);
        BAS.xright=BAS.x-BAS.dir*BAS.d0;
        BAS.fright=f3_2(BAS.xright,N,vb,Beta);    

        BAS.x=BAS.x-BAS.step*BAS.dir*sign(BAS.fleft-BAS.fright);

        BAS.f = f3_2(BAS.x,N,vb,Beta);

        %%%%%%%%%%
        if BAS.f<BAS.fbest
            BAS.xbest=BAS.x;
            BAS.fbest=BAS.f;        
        end
        %%%%%%%%%%%
        BAS.x_store=cat(2,BAS.x_store,[i;BAS.x;BAS.f]);
        BAS.fbest_store=[BAS.fbest_store;BAS.fbest];
        BAS.xbest_store(i,:) = BAS.xbest; 
        BAS.x_store2(:,i) = [i;BAS.x;BAS.f];
        %%%%%%%%%%%
        if(BAS.fbest >= 0.4)
            BAS.step=BAS.step;
        else
            BAS.step=BAS.step*BAS.eta;
        end
    end
    store(j,:) = BAS.xbest;
    store2(j,:) = BAS.xbest-[45;75;-30];
end


err_pitch = sqrt(sum(store2(:,1).^2)/100);
err_roll = sqrt(sum(store2(:,2).^2)/100);
err_yaw = sqrt(sum(store2(:,3).^2)/100);

err_pitch2 = sum(abs(store2(:,1)))/100;
err_roll2 = sum(abs(store2(:,2)))/100;
err_yaw2 = sum(abs(store2(:,3)))/100;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%æ•°æ®æ˜¾ç¤ºéƒ¨åˆ†
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);clf(1);
subplot(3,1,1);hold on;
plot(store2(:,1),'k');legend('Îó²î');
ylabel('¸©Ñö½ÇÎó²î(¡ã)');xlabel('ÊµÑé´ÎÊı');
title("×ËÌ¬Îó²î");grid on;
subplot(3,1,2);hold on;
plot(store2(:,2),'k');legend('Îó²î');
ylabel('ºá¹ö½ÇÎó²î(¡ã)');xlabel('ÊµÑé´ÎÊı');
title("×ËÌ¬Îó²î");grid on;
subplot(3,1,3);hold on;
plot(store2(:,3),'k');legend('Îó²î');
ylabel('Æ«º½½ÇÎó²î(¡ã)');xlabel('ÊµÑé´ÎÊı');
title("×ËÌ¬Îó²î");grid on;

