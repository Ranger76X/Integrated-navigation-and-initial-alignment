function [qbn,qbb]=AttitudeUpdate(qbn0,v0,BLH0,Deltatheta0,Deltatheta,Deltat,WGS84)
    %��̬��Ԫ������
    %���루����Ԫ��̬��Ԫ��������Ԫ�ٶȡ�����Ԫλ�á�����Ԫ�������������Ԫ�����������Ԫ��ʱ������
    %���������Ԫ��̬��Ԫ����
    RM=WGS84.a*(1-WGS84.e2)/(1-WGS84.e2*sin(BLH0(1))*sin(BLH0(1)))^(3/2);%������������Ȧ�뾶(phi(tk-1))
    RN=WGS84.a/sqrt(1-WGS84.e2*sin(BLH0(1))*sin(BLH0(1)));%��������î��Ȧ�뾶(BLH0(1))
    omegaen0=[v0(2)/(RN+BLH0(3));-v0(1)/(RM+BLH0(3));-v0(2)*tan(BLH0(1))/(RN+BLH0(3))];%��һ��Ԫ��λ������
    omegaie0=[WGS84.we*cos(BLH0(1));0;-WGS84.we*sin(BLH0(1))];%��һ��Ԫ������ת���ٶ�

    Phi=Deltatheta+1/12*cross(Deltatheta0,Deltatheta);
    qbb=[cos(0.5*norm(Phi));sin(0.5*norm(Phi))/(0.5*norm(Phi))*0.5*Phi];
    si=(omegaen0+omegaie0)*Deltat;

    qnn=[1-1/8*norm(si)*norm(si);-1/2*si];
    temp=quatmul(qnn,qbn0);%��ʱ����

%     qbn=quatmul(temp,qbb);%��̬��Ԫ���˷�
 
    qbn=quatmul(qbn0,qbb);%��̬��Ԫ���˷�

    qbnValue=sqrt(qbn(1)*qbn(1)+qbn(2)*qbn(2)+qbn(3)*qbn(3)+qbn(4)*qbn(4));

    for i=1:4
        qbn(i)=qbn(i)/qbnValue;%��һ��
    end
end