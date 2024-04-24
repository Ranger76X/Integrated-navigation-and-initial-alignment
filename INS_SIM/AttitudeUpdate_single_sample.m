function [qbn,qbb]=AttitudeUpdate_single_sample(qbn0,Deltatheta0,Deltatheta)
    %姿态四元数更新
    %输入（上历元姿态四元数、上历元陀螺输出、本历元陀螺输出）
    %输出（本历元姿态四元数）
    %无地球物理量，单子样算法。
    
    Phi = Deltatheta+1/12*cross(Deltatheta0,Deltatheta);
    qbb=[cos(0.5*norm(Phi));sin(0.5*norm(Phi))/(0.5*norm(Phi))*0.5*Phi];
 
    qbn=quatmul(qbn0,qbb);%姿态四元数乘法

    qbnValue=sqrt(qbn(1)*qbn(1)+qbn(2)*qbn(2)+qbn(3)*qbn(3)+qbn(4)*qbn(4));

    for i=1:4
        qbn(i)=qbn(i)/qbnValue;%归一化
    end
end