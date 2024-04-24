function [v]=VelocityUpdate3(v0,Deltat,Cbn0,Cbn0_pre,acc,acc_pre)
    %速度更新函数(北东地）
    %输入（上历元速度(北东地）、上历元位置、历元时间间隔、上历元姿态矩阵、本历元加速度计输出)
    %输出（本历元速度）
    format long
    gl = [0;0;9.8];
  
    acc = Cbn0*acc+gl;
    acc_pre = Cbn0_pre*acc_pre+gl;

    DeltaVfn=(acc_pre+acc)/2*Deltat;

    v=v0+DeltaVfn;
end