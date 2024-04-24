function [qbn,qbb]=AttitudeUpdate_qua(qbn0,gyr,Ts)
    %姿态四元数更新
    %输入（上历元姿态四元数、本历元陀螺输出、时间间隔）
    %输出（本历元姿态四元数）
    %四元数角积分
    
    P = gyr(1) * Ts;
    Q = gyr(2) * Ts;
    R = gyr(3) * Ts;
    
    OMEGA = zeros(4);
    OMEGA(1,1:4) = 0.5 * [0 R -Q P];
    OMEGA(2,1:4) = 0.5 * [-R 0 P Q];
    OMEGA(3,1:4) = 0.5 * [Q -P 0 R];
    OMEGA(4,1:4) = 0.5 * [-P -Q -R 0];
    
    v = norm(gyr) * Ts;
    
    if v~=0
        qbn = (cos(v/2) * eye(4)+2/v * sin(v/2) * OMEGA) * qbn0;
        qbn = qbn ./ norm(qbn);
    end
end