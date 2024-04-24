function [qbn]=EulerToQuaternion(Euler)
%��ŷ���ǣ����������������,rad��ת������̬��Ԫ��
roll=Euler(1);
pitch=Euler(2);
yaw=Euler(3);
qbn(1,1)=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
qbn(2,1)=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
qbn(3,1)=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
qbn(4,1)=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
end