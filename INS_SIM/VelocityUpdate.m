function [v]=VelocityUpdate(v0,BLH0,Deltat,Cbn0,Deltav0,Deltav,Deltatheta0,Deltatheta,WGS84)
%速度更新函数(北东地）
%输入（上历元速度(北东地）、上历元位置、历元时间间隔、重力加速度、上历元姿态矩阵、上历元加速度计输出、本历元加速度计输出、上历元陀螺输出、
%本历元陀螺输出）
%输出（本历元速度）
format long
RM=WGS84.a*(1-WGS84.e2)/(1-WGS84.e2*sin(BLH0(1))*sin(BLH0(1)))^(3/2);%载体所在子午圈半径(phi(tk-1))
RN=WGS84.a/sqrt(1-WGS84.e2*sin(BLH0(1))*sin(BLH0(1)));%载体所在卯酉圈半径(BLH0(1))
omegaen0=[v0(2)/(RN+BLH0(3));-v0(1)/(RM+BLH0(3));-v0(2)*tan(BLH0(1))/(RN+BLH0(3))];
omegaie0=[WGS84.we*cos(BLH0(1));0;-WGS84.we*sin(BLH0(1))];
g=Calculateg(BLH0);
gl=[0;0;g];

DeltaVgk=(gl-cross((2*omegaie0+omegaen0),v0))*Deltat;
DeltaVfb=Deltav+1/2*cross(Deltatheta,Deltav)+1/12*(cross(Deltatheta0,Deltav)+cross(Deltav0,Deltatheta));
Xi=(omegaen0+omegaie0)*Deltat;
Xix=askew(Xi);
DeltaVfn=(eye(3)-0.5*Xix)*Cbn0*DeltaVfb;
v=v0+DeltaVgk+DeltaVfn;
end