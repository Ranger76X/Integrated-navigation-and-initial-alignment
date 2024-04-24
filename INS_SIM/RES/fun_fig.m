clc %清除命令窗口的内容
clear %清除工作空间的所有变量
clear all %清除工作空间的所有变量，函数，和MEX文件
clf %清除当前的Figure
close %关闭当前的Figure窗口
close all %关闭所有的Figure窗口

x=-1:0.001:1;
y1=abs(x).^2; 
y2=abs(x).^(1/4);
y3=abs(x).^(1/8);
y4=abs(x).^(1/16);
figure(1);
plot(x,y1); 
legend('f(x)');
ylabel('y');xlabel('x');
title("f(x)函数图像");grid on;

figure(2);
plot(x,y2,'-.'); 
hold on;
plot(x,y3,'.'); 
hold on;
plot(x,y4); 
legend('n=4','n=8','n=16');
ylabel('y');xlabel('x');
title("f(x,n)函数图像(归一化)");grid on;