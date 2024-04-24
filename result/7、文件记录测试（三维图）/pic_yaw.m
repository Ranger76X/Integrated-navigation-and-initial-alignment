clc;
clear;
close all;
xlsread("PIC_YAW.xlsx");

% x=40:70;
% y=20:50;
% zz=ans(20:50,40:70).^16;

x=1:90;
y=1:360;
zz=ans.^16;

[xx,yy]=meshgrid(x,y);
C = zz;

s=surf(xx,yy,zz,C);
s.EdgeColor = 'none';
colorbar
hold on;
h1 = plot3(56,36,ans(36,56).^16,'x','Color','r','MarkerSize',10);
h1.LineWidth = 3;
legend([h1],'��Сֵ');
ylabel('�����(��)','FontSize',15);
xlabel('������(��)','FontSize',15);
zlabel('Wahba����ֵ','FontSize',15);
% title('ƫ���ǹ̶�ʱWahba����ֵ�ֲ����(n=4)');