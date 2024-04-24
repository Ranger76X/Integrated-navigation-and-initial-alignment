clc;
clear;
close all;
xlsread("PIC_ROL.xlsx");

x=40:70;
y=40:80;
zz=ans(40:80,40:70).^16;

% x=1:360;
% y=1:90;
% zz=ans.^16;

[xx,yy]=meshgrid(x,y);
C = zz;

s=surf(xx,yy,zz,C);
s.EdgeColor = 'none';
colorbar
hold on;
h1 = plot3(66,56,ans(56,66).^16,'x','Color','r','MarkerSize',10);
h1.LineWidth = 3;
legend([h1],'最小值');
ylabel('俯仰角(°)');xlabel('偏航角(°)');zlabel('Wahba函数值');
% title('横滚角固定时Wahba函数值分布情况(n=4)');



% ylabel('俯仰角(°)');xlabel('偏航角(°)');zlabel('Wahba函数值');
% title('横滚角固定时Wahba函数值分布情况');
