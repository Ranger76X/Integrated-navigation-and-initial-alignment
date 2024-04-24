function E = Quaternion2Euler(Q)
% 四元数转欧拉角
% 东北天坐标系，旋转顺序Z，X，Y（312）
% 输出角度单位rad

pitch = asin(2*(Q(3)*Q(4)+Q(1)*Q(2)));
row = -atan2(2*(Q(2)*Q(4)-Q(1)*Q(3)),Q(1)*Q(1)-Q(2)*Q(2)-Q(3)*Q(3)+Q(4)*Q(4));
yaw = -atan2(2*(Q(2)*Q(3)-Q(1)*Q(4)),Q(1)*Q(1)-Q(2)*Q(2)+Q(3)*Q(3)-Q(4)*Q(4));

%      x    y   z
% E = [pitch,row,yaw]*180/pi;
E = [pitch,row,yaw];

end 