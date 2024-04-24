function XYZ = B_XYZ2XYZ(B_XYZ,B_L,B_B,B_H,B_YAW,WGS84)
    %发射坐标系转地心地固坐标系
    %输入（发射坐标系XYZ,发射点经度,发射点纬度,发射点高度,射向角(弧度制),WGS84地球模型）
    %输出载体位置（地心地固坐标系XYZ）
   
   
    R = WGS84.R + B_H;     
    
    %发射点卯酉圈半径
    N = WGS84.a / sqrt(1-(WGS84.e2*sin(B_B)*sin(B_B)));
    %发射点地心纬度
    phi = atan((N*(1-WGS84.e2)+B_H)*tan(B_B)/(N+B_H));
    %发射点地心半径
    R0 = [-R*sin(B_B-phi)*cos(B_YAW);R*cos(B_B-phi);R*sin(B_B-phi)*sin(B_YAW);];
    
    M1 = [
            1     0          0;
            0  cos(-B_B) sin(-B_B);
            0 -sin(-B_B) cos(-B_B);
         ];
    M2 = [
            cos(pi/2 + B_YAW)    0    -sin(pi/2 + B_YAW);
                    0            1              0;
            sin(pi/2 + B_YAW)    0     cos(pi/2 + B_YAW);
         ];
    M3 = [
            cos(pi/2 - B_L)  sin(pi/2 - B_L) 0;
           -sin(pi/2 - B_L)  cos(pi/2 - B_L) 0;
                0                0           1;
         ];
     A = M3*M1*M2;
     
     XYZ = A*B_XYZ+A*R0;



end