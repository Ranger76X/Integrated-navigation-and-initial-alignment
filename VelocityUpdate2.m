function [v]=VelocityUpdate2(v0,XYZ,Deltat,Cbn0,Deltav,WGS84)
    %速度更新函数(北东地）
    %输入（上历元速度(北东地）、上历元位置、历元时间间隔、上历元姿态矩阵、本历元加速度计输出)
    %输出（本历元速度）
    format long
    
    gl = [0;0;WGS84.ge*(1-2*XYZ(3)/WGS84.R)];

    DeltaVgk=(gl)*Deltat;

    DeltaVfn=Cbn0*Deltav;

    v=v0-DeltaVgk+DeltaVfn;
end