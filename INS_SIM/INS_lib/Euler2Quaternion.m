function Q = Euler2Quaternion(pitch,roll,yaw)
% 欧拉角转四元数
% 东北天坐标系，旋转顺序Z，X，Y（312）
% 输入角度单位rad

% pitch = pitch/180*pi;       %x
% roll = roll/180*pi;         %y
% yaw = yaw/180*pi;           %z


Q(1) = cos(yaw/2) * cos(pitch/2) * cos(roll/2) - sin(yaw/2) * sin(pitch/2) * sin(roll/2);
Q(2) = cos(yaw/2) * sin(pitch/2) * cos(roll/2) - sin(yaw/2) * cos(pitch/2) * sin(roll/2);
Q(3) = sin(yaw/2) * sin(pitch/2) * cos(roll/2) + cos(yaw/2) * cos(pitch/2) * sin(roll/2);
Q(4) = sin(yaw/2) * cos(pitch/2) * cos(roll/2) + cos(yaw/2) * sin(pitch/2) * sin(roll/2);

end 