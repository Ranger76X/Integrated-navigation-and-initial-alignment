function [BLH]=LocationUpdata3(BLH0,v,v0,Deltat,WGS84)
%位置更新
%输入（上个历元的位置、本历元速度、上个历元速度、历间时间间隔）
%输出载体位置（高程经纬度）
RM=WGS84.a*(1-WGS84.e2)/(1-WGS84.e2*sin(BLH0(1))*sin(BLH0(1)))^(3/2);%载体所在子午圈半径(phi(tk-1))
BLH(3,1)=BLH0(3)-1/2*(v(3)+v0(3))*Deltat;
h=1/2*(BLH0(3)+BLH(3));
BLH(1,1)=BLH0(1)+1/2*(v(1)+v0(1))/(RM+h)*Deltat;
phi=1/2*(BLH(1)+BLH0(1));
RN=WGS84.a/sqrt(1-WGS84.e2*sin(phi)*sin(phi));%载体所在卯酉圈半径(phi)
BLH(2,1)=BLH0(2)+1/2*(v(2)+v0(2))/((RN+h)*cos(phi))*Deltat;
end