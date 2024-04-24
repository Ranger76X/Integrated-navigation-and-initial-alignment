clc;
clear;
close all;
xlsread("PIC_PIT.xlsx");

x=50:80;
y=20:50;
zz=ans(20:50,50:80).^16;

% x=1:360;
% y=1:360;
% zz=ans.^16;

[xx,yy]=meshgrid(x,y);
C = zz;

s=surf(xx,yy,zz,C);
hold on;
h1 = plot3(66,36,ans(36,66).^16,'x','Color','r','MarkerSize',10);
h1.LineWidth = 3;
s.EdgeColor = 'none';
colorbar
legend([h1],'��Сֵ');
ylabel('�����(��)');xlabel('ƫ����(��)');zlabel('Wahba����ֵ');
% title('�����ǹ̶�ʱWahba����ֵ�ֲ����(n=4)');

figure(2);
contour(xx,yy,zz);
% c.EdgeColor = 'none';
colorbar;
ylabel('�����(��)');xlabel('ƫ����(��)');zlabel('Wahba����ֵ');
title('�����ǹ̶�ʱWahba����ֵ�ֲ����');