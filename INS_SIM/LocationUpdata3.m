function [BLH]=LocationUpdata3(BLH0,v,v0,Deltat,WGS84)
%λ�ø���
%���루�ϸ���Ԫ��λ�á�����Ԫ�ٶȡ��ϸ���Ԫ�ٶȡ�����ʱ������
%�������λ�ã��߳̾�γ�ȣ�
RM=WGS84.a*(1-WGS84.e2)/(1-WGS84.e2*sin(BLH0(1))*sin(BLH0(1)))^(3/2);%������������Ȧ�뾶(phi(tk-1))
BLH(3,1)=BLH0(3)-1/2*(v(3)+v0(3))*Deltat;
h=1/2*(BLH0(3)+BLH(3));
BLH(1,1)=BLH0(1)+1/2*(v(1)+v0(1))/(RM+h)*Deltat;
phi=1/2*(BLH(1)+BLH0(1));
RN=WGS84.a/sqrt(1-WGS84.e2*sin(phi)*sin(phi));%��������î��Ȧ�뾶(phi)
BLH(2,1)=BLH0(2)+1/2*(v(2)+v0(2))/((RN+h)*cos(phi))*Deltat;
end