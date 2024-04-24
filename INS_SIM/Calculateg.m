function [g]=Calculateg(BLH)
%计算重力加速度g
%{
g1=(WGS84.a*WGS84.ge*cos(BLH(1))*cos(BLH(1))+WGS84.b*WGS84.gp*sin(BLH(1))*sin(BLH(1)))...
    /sqrt(WGS84.a*WGS84.a*cos(BLH(1))*cos(BLH(1))+WGS84.b*WGS84.b*sin(BLH(1))*sin(BLH(1)));
m=WGS84.we*WGS84.we*WGS84.a*WGS84.a*WGS84.b/WGS84.GM;
g=g1*(1-2/WGS84.a*(1+WGS84.f+m-2*WGS84.f*sin(BLH(1))*sin(BLH(1)))*BLH(3)+3/(WGS84.a*WGS84.a)*BLH(3)*BLH(3));
%}
garg=[9.7803267715,0.0052790414,0.0000232718,-0.000003087691089,0.000000004397731,0.000000000000721];
g=garg(1)*(1.0+garg(2)*sin(BLH(1))^2+garg(3)*sin(BLH(1))^4)+(garg(4)+garg(5)*sin(BLH(1))^2)*BLH(3)+garg(6)*BLH(3)^2;   %当地重力加速度 

end