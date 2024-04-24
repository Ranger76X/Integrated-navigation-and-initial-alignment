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
legend([h1],'��Сֵ');
ylabel('������(��)');xlabel('ƫ����(��)');zlabel('Wahba����ֵ');
% title('����ǹ̶�ʱWahba����ֵ�ֲ����(n=4)');



% ylabel('������(��)');xlabel('ƫ����(��)');zlabel('Wahba����ֵ');
% title('����ǹ̶�ʱWahba����ֵ�ֲ����');
