function BLH = XYZ2LLA(XYZ,WGS84)
    %地心地固坐标系转纬经高
    %输入（地心地固坐标系XYZ,WGS84地球模型）
    %输出载体位置（纬经高BLH）
r = sqrt(XYZ(1)*XYZ(1)+XYZ(2)*XYZ(2));
R = sqrt(r*r + XYZ(3)*XYZ(3));
K = R/WGS84.a - ((1 - WGS84.f) / sqrt(1 - WGS84.e2*r*r/(R*R)));
E = WGS84.e2 / (1 + K * sqrt(1 - WGS84.e2*XYZ(3)*XYZ(3)/(R*R)) );


BLH(1) = atan2(XYZ(3),(r*(1-E)));

BLH(2) = atan2(XYZ(2),XYZ(1));
    
% if(XYZ(1)<0)
%     BLH(2) = BLH(2) + pi; 
% elseif(XYZ(1)>0&&XYZ(2)<0)
%     BLH(2) = BLH(2) + 2 * pi; 
% end

N = WGS84.a /sqrt(1 - WGS84.e2 * sin(BLH(1))*sin(BLH(1)));

BLH(3) = sqrt(r*r + (XYZ(3) + N * WGS84.e2 * sin(BLH(1)) ) * 2) - N;



end