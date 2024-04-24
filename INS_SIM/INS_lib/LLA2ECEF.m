%经纬高坐标系—>ECEF坐标系
function [xyz]=LLA2ECEF(LLA)
    LLA(1,:) = LLA(1,:)/180*pi;%纬度
    LLA(2,:) = LLA(2,:)/180*pi;%经度
    [~,N] = size(LLA);
    xyz = zeros(3,N);
    for i = 1:N
        lat = LLA(1,i); 
        lon = LLA(2,i);
        h = LLA(3,i); 
        a = 6378137;
        e = 0.0818191910;
        RN = a/sqrt(1-e^2*sin(lat)^2);
        x = (RN+h)*cos(lat)*cos(lon);
        y = (RN+h)*cos(lat)*sin(lon);
        z = (RN*(1-e^2)+h)*sin(lat);

        xyz(:,i) = [x;y;z];
    end
end