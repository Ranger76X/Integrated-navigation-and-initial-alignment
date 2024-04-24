function [enu_xyz]=ECEF2ENU(lon0,lat0,ecef_xyz)
%ECEF系转导航坐标系ENU
%ecef_xyz：3*N
    lon0 = lon0/180*pi;
    lat0 = lat0/180*pi;
    T11 = -sin(lon0);
    T12 = cos(lon0);
    T13 = 0;
    T21 = -sin(lat0)*cos(lon0);
    T22 = -sin(lat0)*sin(lon0);
    T23 = cos(lat0);
    T31 = cos(lat0)*cos(lon0);
    T32 = cos(lat0)*sin(lon0);
    T33 = sin(lat0);
    T = [T11, T12, T13; T21, T22, T23; T31, T32, T33];
    enu_xyz = T*ecef_xyz; 
end
