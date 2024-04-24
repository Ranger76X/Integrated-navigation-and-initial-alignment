function [qbn,qbb]=AttitudeUpdate(qbn0,v0,BLH0,Deltatheta0,Deltatheta,Deltat,WGS84)
    %姿态四元数更新
    %输入（上历元姿态四元数、上历元速度、上历元位置、上历元陀螺输出、本历元陀螺输出、历元间时间间隔）
    %输出（本历元姿态四元数）
    RM=WGS84.a*(1-WGS84.e2)/(1-WGS84.e2*sin(BLH0(1))*sin(BLH0(1)))^(3/2);%载体所在子午圈半径(phi(tk-1))
    RN=WGS84.a/sqrt(1-WGS84.e2*sin(BLH0(1))*sin(BLH0(1)));%载体所在卯酉圈半径(BLH0(1))
    omegaen0=[v0(2)/(RN+BLH0(3));-v0(1)/(RM+BLH0(3));-v0(2)*tan(BLH0(1))/(RN+BLH0(3))];%上一历元的位置速率
    omegaie0=[WGS84.we*cos(BLH0(1));0;-WGS84.we*sin(BLH0(1))];%上一历元地球自转角速度

    Phi=Deltatheta+1/12*cross(Deltatheta0,Deltatheta);
    qbb=[cos(0.5*norm(Phi));sin(0.5*norm(Phi))/(0.5*norm(Phi))*0.5*Phi];
    si=(omegaen0+omegaie0)*Deltat;

    qnn=[1-1/8*norm(si)*norm(si);-1/2*si];
    temp=quatmul(qnn,qbn0);%临时变量

%     qbn=quatmul(temp,qbb);%姿态四元数乘法
 
    qbn=quatmul(qbn0,qbb);%姿态四元数乘法

    qbnValue=sqrt(qbn(1)*qbn(1)+qbn(2)*qbn(2)+qbn(3)*qbn(3)+qbn(4)*qbn(4));

    for i=1:4
        qbn(i)=qbn(i)/qbnValue;%归一化
    end
end