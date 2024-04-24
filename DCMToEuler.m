function [Euler]=DCMToEuler(Cbn)
%方向余弦转欧拉角（roll,pitch,yaw,rad)
Euler(1,1)=atan2(Cbn(3,2),Cbn(3,3));%-pi<roll<pi)
Euler(2,1)=atan(-Cbn(3,1)/sqrt(Cbn(3,2)*Cbn(3,2)+Cbn(3,3)*Cbn(3,3)));%(-pi/2<pitch<pi/2)
Euler(3,1)=atan2(Cbn(2,1),Cbn(1,1));%(-pi<yaw<pi)
end