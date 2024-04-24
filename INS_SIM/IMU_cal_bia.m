function [acc_bia,gyr_bia] = IMU_cal_bia(acc,gyr,N)

acc_bia = [sum(acc(1,1:N))/N;sum(acc(2,1:N))/N;sum(acc(3,1:N))/N];
gyr_bia = [sum(gyr(1,1:N))/N;sum(gyr(2,1:N))/N;sum(gyr(3,1:N))/N];

end