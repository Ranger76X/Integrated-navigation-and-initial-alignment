 function [R,U,S,V] = SVD_QUEST(fb, mb, fn, mn, wf, wm)
% q = SVD_QUEST(Sensors, Parameters)
% Function implements QUEST algorithm using measurements
% from three-component accelerometer with orthogonal axes and vector
% magnetometer
%
%   Input arguments:
%   fb  - Acceleration vector in body frame [3x1]
%   mb  - Magnetic field vector in body frame [3x1]
%   fn  - Gravity vector in navigation frame [3x1]
%   mn  - Magetic field vector in navigation frame [3x1]
%
%   Output arguments:
%   Cnb - estimated Direction Cosines Matrix (DCM)   B2N

F = wf*fn*fb' + wm*mn*mb';        %双矢量whaba问题B2N

[U,S,V] = svd(F);

U = U*[1,0,0;0,1,0;0,0,det(U)];
V = V*[1,0,0;0,1,0;0,0,det(V)];
S = S*[1,0,0;0,1,0;0,0,det(U*V)];

R = U*V';
% R = R';         %B2N 2 N2B

 end