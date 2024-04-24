clc;clear;close all;
addpath('./data');
addpath('./INS_lib');
addpath('./Kalman_f');
addpath('./Att_init_lib');
load("随机游走449473373032.mat");
Cnb2 = q2mat(Euler2Quaternion(44.9812/180*pi,76.1276/180*pi,-29.8432/180*pi));

% Cnb2 = q2mat(Euler2Quaternion(45/180*pi,75/180*pi,-30/180*pi));

twoKp = 25;
twoKi = 0.35;
integralFBx = 0;
integralFBy = 0;
integralFBz = 0;
qbb2(:,1) = [1;0;0;0];
Cbb2(:,:,1) = Quaternion2ACM(qbb(:,1));
Eulerbb2(:,1) = [0;0;0];
gx = zeros(3,D_N);

vb2(:,1) = [0;0;0];
for i2 = 2:D_N-1 
    
    %--------------------------------1.濮挎?佺煩闃佃В绠?------------------------------
    D_GYR2(:,i2-1) = D_GYR(:,i2-1)+gx(:,i2-1);
    D_GYR2(:,i2) = D_GYR(:,i2)+gx(:,i2);
    Delta_sita2(:,i2) = (D_GYR2(:,i2-1))*D_T+(D_GYR2(:,i2)-D_GYR2(:,i2-1))/2*D_T;
    qbb2(:,i2) = AttitudeUpdate_single_sample(qbb2(:,i2-1), Delta_sita2(:,i2-1),Delta_sita2(:,i2));
    Eulerbb2(:,i2) = Quaternion2Euler(qbb2(:,i2))/pi*180;
    Cbb2(:,:,i2) = Quaternion2ACM(qbb2(:,i2));
    %--------------------------------2.閫熷害鏇存柊---------------------------------
    vb2(:,i2) = VelocityUpdate_SVD(vb2(:,i2-1),D_T,Cbb2(:,:,i2),Cbb2(:,:,i2-1),D_ACC(:,i2),D_ACC(:,i2-1));
    
    Alpha(:,i2) = Cnb2*vb(:,i2);
    Alpha2(:,i2) = Cnb2*vb2(:,i2);
    Alpha3(:,i2) = Alpha2(:,i2)/norm(Alpha2(:,i2));
    Beta2(:,i2) = Beta(:,i2)/norm(Beta(:,i2));
    err(:,i2) = cross(Beta2(:,i2),Alpha3(:,i2));
    if(twoKi > 0.0) 
        integralFBx = integralFBx + twoKi * err(1,i2) * 0.005;	
        integralFBy = integralFBy + twoKi * err(2,i2) * 0.005;
        integralFBz = integralFBz + twoKi * err(3,i2) * 0.005;
        gx(1,i2) = gx(1,i2) + integralFBx;	
        gx(2,i2) = gx(2,i2) + integralFBy;
        gx(3,i2) = gx(3,i2) + integralFBz;
    else 
        integralFBx = 0.0;	
        integralFBy = 0.0;
        integralFBz = 0.0;
    end
    
    gx(1,i2) = gx(1,i2) + twoKp * err(1,i2);	
    gx(2,i2) = gx(2,i2) + twoKp * err(3,i2);
    gx(3,i2) = gx(3,i2) + twoKp * err(3,i2);
    
    
end


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %BAS鍒濆鍖栭儴鍒?1
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % roll = 0;
% roll = atan2(Delta_sita(3,2),Delta_sita(1,2))/pi*180+180;
roll = 74;
% % roll = atan2(D_GYR(3,2),D_GYR(1,2))/pi*180+180;
% % roll = atan(Delta_sita(3,2)/Delta_sita(1,2))/pi*180;
% % roll = atan(D_GYR(3,1)/D_GYR(1,1))/pi*180;
% 
% 
N = 1000;%闇?瑕佺殑瑙傛祴鐭㈤噺瀵规暟
BAS.eta = 0.9;
BAS.c = 1;                  %ratio between step and d0
BAS.step = 1;               %initial step set as the largest input range
BAS.n = D_N;                %iterations
BAS.k = 3;                  %space dimension
BAS.x = [44.9812;76.1276;-29.8432];        %intial value
BAS.xbest = BAS.x;
BAS.f = f3(BAS.x,N,vb2,Beta);
BAS.fbest = BAS.f;
BAS.fbest_store = BAS.fbest;
BAS.x_store = [0;BAS.x;BAS.fbest];


for i = 1:BAS.n
    BAS.d0=BAS.step/BAS.c;
    BAS.dir=rands(BAS.k,1);
    BAS.dir=BAS.dir/(eps+norm(BAS.dir));    
    
    BAS.xleft=BAS.x+BAS.dir*BAS.d0;
    BAS.fleft=f3(BAS.xleft,N,vb2,Beta);
    BAS.xright=BAS.x-BAS.dir*BAS.d0;
    BAS.fright=f3(BAS.xright,N,vb2,Beta);    
    
    BAS.x=BAS.x-BAS.step*BAS.dir*sign(BAS.fleft-BAS.fright);
    
    BAS.f = f3(BAS.x,N,vb2,Beta);
    
    %%%%%%%%%%
    if BAS.f<BAS.fbest
        BAS.xbest=BAS.x;
        BAS.fbest=BAS.f;        
    end
    %%%%%%%%%%%
    BAS.x_store=cat(2,BAS.x_store,[i;BAS.x;BAS.f]);
    BAS.fbest_store=[BAS.fbest_store;BAS.fbest];
    BAS.xbest_store(i,:) = BAS.xbest; 
%     phi(i) = BAS.xbest;
    display([num2str(i),':xbest=[',num2str(BAS.xbest'),'],fbest=',num2str(BAS.fbest)])
    %%%%%%%%%%%
    if(BAS.fbest >= 0.01)
        BAS.step=BAS.step;
    else
        BAS.step=BAS.step*BAS.eta;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%鏁版嵁鏄剧ず閮ㄥ垎
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% figure(1);
% clf(1);
% plot(BAS.x_store(1,:),BAS.x_store(end,:),'r-o');
% hold on;
% plot(BAS.x_store(1,:),BAS.fbest_store,'b-.');
% xlabel('iteration');
% ylabel('minimum value');

figure(1);
clf(1);
plot(BAS.x_store(2,:));
hold on
plot(BAS.xbest_store(:,1));
legend('BAS','BASR','SVD');
title('弹道初始俯仰姿态对准结果');

figure(2);
clf(2);
plot(BAS.x_store(3,:));
hold on
plot(BAS.xbest_store(:,2));
hold on;
title('弹道初始横滚姿态对准结果');

figure(3);
clf(3);
plot(BAS.x_store(4,:));
hold on
plot(BAS.xbest_store(:,3));
title('弹道初始偏航姿态对准结果');
display([num2str(i),':real=[',num2str([45;75;-30]'),'],fbest=',num2str(f3([45,75,-30],N,vb,Beta))])
bb = [Y(2:end,4)-Y(1,4),Y(2:end,5)-Y(1,5),Y(2:end,6)-Y(1,6)]';

figure(4);
plot(Alpha2(1,:));
hold on;
plot(Beta(1,:));
hold on;
plot(bb(1,:));
hold on;
plot(Alpha(1,:));
legend('补偿后','弹道','补偿前');

figure(5);
plot(Alpha2(2,:));
hold on;
plot(Beta(2,:));
hold on;
plot(bb(2,:));
hold on;
plot(Alpha(2,:));
legend('补偿后','弹道','补偿前');

figure(6);
plot(Alpha2(3,:));
hold on;
plot(Beta(3,:));
hold on;
plot(bb(3,:));
hold on;
plot(Alpha(3,:));
legend('补偿后','弹道','补偿前');

ERR = Alpha2-Beta(:,2:end);
ERR2 = Alpha2(:,2:end)-bb(:,1:end-1);
ERR3 = Alpha(:,2:end)-bb(:,1:end-1);

figure (7);
plot(gx(1,1:end-1));
figure(8);
plot(gx(2,1:end-1));
figure(9);
plot(gx(3,1:end-1));



% roll = atan2(Delta_sita(3,2),Delta_sita(1,2))/pi*180+180;