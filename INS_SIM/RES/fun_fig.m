clc %�������ڵ�����
clear %��������ռ�����б���
clear all %��������ռ�����б�������������MEX�ļ�
clf %�����ǰ��Figure
close %�رյ�ǰ��Figure����
close all %�ر����е�Figure����

x=-1:0.001:1;
y1=abs(x).^2; 
y2=abs(x).^(1/4);
y3=abs(x).^(1/8);
y4=abs(x).^(1/16);
figure(1);
plot(x,y1); 
legend('f(x)');
ylabel('y');xlabel('x');
title("f(x)����ͼ��");grid on;

figure(2);
plot(x,y2,'-.'); 
hold on;
plot(x,y3,'.'); 
hold on;
plot(x,y4); 
legend('n=4','n=8','n=16');
ylabel('y');xlabel('x');
title("f(x,n)����ͼ��(��һ��)");grid on;