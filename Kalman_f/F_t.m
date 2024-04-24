function [Ft,Ft2]=F_t(wie,v,h,cbn,f_c,RM,RN,BLH_B)
%
%%%卡尔曼滤波的F阵%%%% 
    M9=zeros(3,3);
    L=BLH_B*pi/180;
    M1=[0 wie*sin(L)+(v(2)*tan(L))/(RN+h) -(wie*cos(L)+v(2)/(RN+h));
        -(wie*sin(L)+(v(2)*tan(L))/(RN+h)) 0 -v(1)/(RM+h);
        wie*cos(L)+v(2)/(RN+h) v(1)/(RM+h) 0];
    M2=[0 -1/(RM+h) 0;
        1/(RN+h) 0 0;
        tan(L)/(RN+h) 0 0];
    M3=[0 0 -v(1)/((RM+h)^2);
        -wie*sin(L) 0 -v(2)/((RN+h)^2);
        wie*cos(L)+(v(2)*(sec(L))^2)/(RN+h) 0 -v(2)*tan(L)/((RN+h)^2)];
    M5=[(v(1)*tan(L)-v(3))/(RM+h) 2*wie*sin(L)+v(2)*tan(L)/(RN+h) -(2*wie*cos(L)+v(2)/(RN+h));
        -2*(wie*sin(L)+v(2)*tan(L)/(RN+h)) -v(3)/(RM+h) -v(1)/(RM+h);
        2*(wie*cos(L)+v(2)/(RN+h)) 2*v(1)/(RM+h) 0 ];
    M6=[2*v(1)*wie+v(3)*wie*sin(L)+(v(1)*v(2)*sec(L)^2)/(RN+h) 0 v(2)*(v(3)-v(1)*tan(L))/((RN+h)^2);
        -(v(2)*(2*wie*cos(L)+(v(2)*sec(L)^2))/(RN+h)) 0 v(1)*v(3)/((RM+h)^2)+v(2)^2*tan(L)/((RN+h)^2);
        -2*v(2)*wie*sin(L) 0 -(v(1)^2/((RN+h)^2)+v(2)^2/((RM+h)^2))];
    M4=[0 -f_c(3) f_c(1);
        f_c(3) 0 -f_c(2);
        -f_c(1) f_c(2) 0];
    M7=[0 1/(RM+h) 0;
        sec(L)/(RN+h) 0 0;
        0 0 1];
    M8=[0 0 -v(1)/((RM+h)^2);
        v(2)*sec(L)*tan(L)/(RN+h) 0 -v(2)*sec(L)/(RN+h);
        0 0 0];
   
    Ft2=[M1 M2 M3 -cbn M9;
        M4 M5 M6 M9 cbn;
        M9 M7 M8 M9 M9;
    zeros(6,15)];

Ft=[M5 M6 cbn;
    M7 M8 M9;
    zeros(3,9)];


end