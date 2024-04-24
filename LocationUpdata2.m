function [XYZ]=LocationUpdata2(XYZ0,v,Deltat)
    %位置更新
    %输入（上个历元的位置、本历元速度、历间时间间隔）
    %输出载体位置（XYZ）
    
    XYZ(1) = XYZ0(1) - v(1) * Deltat;
%     *R/(R+yn(2));
    XYZ(2) = XYZ0(2) + v(2) * Deltat;
    XYZ(3) = XYZ0(3) - v(3) * Deltat;  
end