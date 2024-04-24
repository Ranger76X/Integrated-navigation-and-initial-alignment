function [qbn]=EulerToQuaternion(Euler)
%将欧拉角（横滚，俯仰，航向,rad）转换成姿态四元数
roll=Euler(1);
pitch=Euler(2);
yaw=Euler(3);
qbn(1,1)=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
qbn(2,1)=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
qbn(3,1)=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
qbn(4,1)=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
end