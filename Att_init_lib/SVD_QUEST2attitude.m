function [qua,eul,R] = SVD_QUEST2attitude(N,VEC1,VEC2,PRAM1,PRAM2,R)

qua = zeros(N,4);
eul = zeros(N,3);
for i = 1:N
    R(:,:,i) = SVD_QUEST(VEC1(i,:)',VEC2(i,:)',VEC1(1,:)',VEC2(1,:)',PRAM1,PRAM2);
    qua(i,:) = nomalize(dcm2quat(R(:,:,i)));                %（注意！此处为MATLAB自带函数）
    [r1,r2,r3] = quat2angle(qua(i,:),'ZYX');                %（注意！此处为MATLAB自带函数）
    eul(i,:) = [r1,r2,r3]*180/pi;                  
end

figure (10);
subplot(3,1,1);
hold on;
h1 = plot(eul(:,1),'b');
% legend([h1],'双矢量定姿');
title("ROLL");

subplot(3,1,2);
hold on;
h2 = plot(eul(:,2),'b');
% legend([h2],'双矢量定姿');
title("PITCH");

subplot(3,1,3);
hold on;
h3 = plot(eul(:,end),'b');
% legend([h3],'双矢量定姿');
title("YAW");

end