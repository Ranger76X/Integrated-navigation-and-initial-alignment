function [yn,k1,check]=AttitudeUpdate_eul(yn,gyr,h)
    %姿态四元数更新
    %输入（上历元姿态四元数、本历元陀螺输出、时间间隔）
    %输出（本历元姿态四元数）
    %角积分
    x = gyr(1);y = gyr(2);z = gyr(3);
    k1 =[
%           yn(1)%d pitch
%           yn(2)%d roll
%           yn(3)%d yaw
          z*sin(yn(2)) + x*cos(yn(2))
          y - tan(yn(1)) * (z*cos(yn(2)) - x*sin(yn(2)))
          1/cos(yn(1)) * (z*cos(yn(2)) - x*sin(yn(2)))
        ];    
    
    k2=[
%          yn(1)+0.5*h*k1(4)
%          yn(2)+0.5*h*k1(5)
%          yn(3)+0.5*h*k1(6)
%          z*sin(yn(2)+0.5*h*k1(5)) + x*cos(yn(2)+0.5*h*k1(5))
%          y-tan(yn(1)+0.5*h*k1(4)) * (z*cos(yn(2)+0.5*h*k1(5))+x*sin(yn(2)+0.5*h*k1(5)))
%          1/cos(yn(1)+0.5*h*k1(4)) * (z*cos(yn(2)+0.5*h*k1(5))+x*sin(yn(2)+0.5*h*k1(5)))
         z*sin(yn(2) + 0.5*h*k1(2)) + x*cos(yn(2) + 0.5*h*k1(2))
         y - tan(yn(1) + 0.5*h*k1(1)) * (z*cos(yn(2) + 0.5*h*k1(2)) - x*sin(yn(2) + 0.5*h*k1(2)))
         1/cos(yn(1) + 0.5*h*k1(1)) * (z*cos(yn(2) + 0.5*h*k1(2)) - x*sin(yn(2) + 0.5*h*k1(2)))
        ];
    
    k3=[
%          yn(1) + 0.5*h*k2(4)
%          yn(2) + 0.5*h*k2(5)
%          yn(3) + 0.5*h*k2(6)
         z*sin(yn(2) + 0.5*h*k2(2)) + x*cos(yn(2) + 0.5*h*k2(2))
         y - tan(yn(1) + 0.5*h*k2(1)) * (z*cos(yn(2) + 0.5*h*k2(2)) - x*sin(yn(2) + 0.5*h*k2(2)))
         1/cos(yn(1) + 0.5*h*k2(1)) * (z*cos(yn(2) + 0.5*h*k2(2)) - x*sin(yn(2) + 0.5*h*k2(2)))
        ];
    k4=[
%          yn(1) + h*k3(4)
%          yn(2) + h*k3(5)
%          yn(3) + h*k3(6)
         z*sin(yn(2) + h*k3(2)) + x*cos(yn(2) + h*k3(2))
         y - tan(yn(1) + h*k3(1)) * (z*cos(yn(2) + h*k3(2)) - x*sin(yn(2) + h*k3(2)))
         1/cos(yn(1) + h*k3(1)) * (z*cos(yn(2) + h*k3(2)) - x*sin(yn(2) + h*k3(2)))
        ];
    
    yn(1) = yn(1) + (k1(1) + 2*k2(1) + 2*k3(1) + k4(1)) * h/6;  %pitch
    yn(2) = yn(2) + (k1(2) + 2*k2(2) + 2*k3(2) + k4(2)) * h/6;  %roll
    yn(3) =  yn(3) + (k1(3) + 2*k2(3) + 2*k3(3) + k4(3)) * h/6; %yaw
    check = (k1(3) + 2*k2(3) + 2*k3(3) + k4(3)) * h/6;

    %     YY(:,n+1)=[
%         yn(1) + (k1(1) + 2*k2(1) + 2*k3(1) + k4(1)) * h/6 %pitch
%         yn(2) + (k1(2) + 2*k2(2) + 2*k3(2) + k4(2)) * h/6 %roll
%         yn(3) + (k1(3) + 2*k2(3) + 2*k3(3) + k4(3)) * h/6 %yaw
% %         yn(4)+ (k1(4)+2*k2(4)+2*k3(4)+k4(4))*h/6 %vx
% %         yn(5)+ (k1(5)+2*k2(5)+2*k3(5)+k4(5))*h/6 %vy
% %         yn(6)+ (k1(6)+2*k2(6)+2*k3(6)+k4(6))*h/6 %vz
%         ];
%      
%      yn=YY(:,n+1);       %第n+1列矩阵
%      n=n+1;
end