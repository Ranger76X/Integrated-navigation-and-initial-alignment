function [Cbn]=QuaternionToDCM(qbn)
%姿态四元数转方向余弦矩阵
Cbn(1,1)=qbn(1)*qbn(1)+qbn(2)*qbn(2)-qbn(3)*qbn(3)-qbn(4)*qbn(4);
Cbn(1,2)=2*(qbn(2)*qbn(3)-qbn(1)*qbn(4));
Cbn(1,3)=2*(qbn(2)*qbn(4)+qbn(1)*qbn(3));
Cbn(2,1)=2*(qbn(2)*qbn(3)+qbn(1)*qbn(4));
Cbn(2,2)=qbn(1)*qbn(1)-qbn(2)*qbn(2)+qbn(3)*qbn(3)-qbn(4)*qbn(4);
Cbn(2,3)=2*(qbn(3)*qbn(4)-qbn(1)*qbn(2));
Cbn(3,1)=2*(qbn(2)*qbn(4)-qbn(1)*qbn(3));
Cbn(3,2)=2*(qbn(3)*qbn(4)+qbn(1)*qbn(2));
Cbn(3,3)=qbn(1)*qbn(1)-qbn(2)*qbn(2)-qbn(3)*qbn(3)+qbn(4)*qbn(4);
end