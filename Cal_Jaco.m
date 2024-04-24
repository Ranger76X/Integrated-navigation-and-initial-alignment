function f = Cal_Jaco(N,roll,pitch,yaw,Alpha,Beta)

     
Cnb_x  =  [
            cos(pitch) sin(pitch)   0     ;
           -sin(pitch) cos(pitch)   0     ;
                0        0          1     ;
          ];%pitch
Cnb_y  =  [
                1        0          0     ;
                0     cos(roll)  sin(roll);
                0    -sin(roll)  cos(roll);
          ];%roll
Cnb_z  =  [
            cos(yaw)     0      -sin(yaw);
                0        1          0    ;
            sin(yaw)     0       cos(yaw);
          ];%yaw  
Cnb = Cnb_y*Cnb_x*Cnb_z;     %312   zxy

d_Cnb_x = [
           -sin(pitch)    cos(pitch)   0;
           -cos(pitch)   -sin(pitch)   0;
               0             0         0;
          ];%pitch 
d_Cnb_y = [
               0         0          0;
               0    -sin(roll)   cos(roll);
               0    -cos(roll)  -sin(roll);
          ];%roll 
d_Cnb_z = [
           -sin(yaw)    0      -cos(yaw);
              0         0         0;
            cos(yaw)    0      -sin(yaw);
         ];%yaw 
%%%%%%%%%%df(x)%%%%%%%%%%%
    A  = [0;0;0];
    for i = 1:N
        
        droll = (d_Cnb_y*Cnb_x*Cnb_z*Beta(:,i))';
        dpitch = (Cnb_y*d_Cnb_x*Cnb_z*Beta(:,i))';
        dyaw = (Cnb_y*Cnb_x*d_Cnb_z*Beta(:,i))';
        Jaco = [droll;dpitch;dyaw];
        
        B = (Alpha(:,i) - Cnb*Beta(:,i));
        A = A + Jaco*B;   
    end
    f = 1/(N) * (A);   %pitch roll yaw
%%%%%%%%%%df(x)%%%%%%%%%%%
end