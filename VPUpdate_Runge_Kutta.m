function [yn]=VPUpdate_Runge_Kutta(yn,acc,Cbn,h,WGS84)
    %速度更新函数(北东地）
    %输入（上历元速度(北东地）、上历元位置、历元时间间隔、上历元姿态矩阵、本历元加速度计输出)
    %输出（本历元速度）
    format long
    
    gl = [0;0;WGS84.ge*(1-2*yn(3)/WGS84.R)];
%     DeltaVgk=(gl)*Deltat;
%     DeltaVfn=Cbn0*Deltav;
%     v=v0+DeltaVgk+DeltaVfn;
    acc = Cbn*acc+gl;
    
    x = acc(1);y = acc(2);z = acc(3);
    k1 =[
          yn(4)               %dx   vx
          yn(5)               %dy   vy
          yn(6)               %dz   vy
          x                   %dvx
          y                   %dvy
          z                   %dvz
        ];    
    
    k2=[
          yn(4)+0.5*h*k1(4)               %dx   vx
          yn(5)+0.5*h*k1(5)               %dy   vy
          yn(6)+0.5*h*k1(6)               %dz   vy
          x                   %dvx
          y                   %dvy
          z                   %dvz
        ];
    
    k3=[
          yn(4)+0.5*h*k2(4)               %dx   vx
          yn(5)+0.5*h*k2(5)               %dy   vy
          yn(6)+0.5*h*k2(6)               %dz   vy
          x                   %dvx
          y                   %dvy
          z                   %dvz
        ];
    k4=[
          yn(4)+h*k3(4)               %dx   vx
          yn(5)+h*k3(5)               %dy   vy
          yn(6)+h*k3(6)               %dz   vy
          x                   %dvx
          y                   %dvy
          z                   %dvz
        ];
    
    yn(1) = yn(1) + (k1(1) + 2*k2(1) + 2*k3(1) + k4(1)) * h/6;  %pitch
    yn(2) = yn(2) + (k1(2) + 2*k2(2) + 2*k3(2) + k4(2)) * h/6;  %roll
    yn(3) =  yn(3) + (k1(3) + 2*k2(3) + 2*k3(3) + k4(3)) * h/6; %yaw
    yn(4) = yn(4) + (k1(4) + 2*k2(4) + 2*k3(4) + k4(4)) * h/6;  %pitch
    yn(5) = yn(5) + (k1(5) + 2*k2(5) + 2*k3(5) + k4(5)) * h/6;  %roll
    yn(6) =  yn63) + (k1(6) + 2*k2(6) + 2*k3(6) + k4(6)) * h/6; %yaw
    
end