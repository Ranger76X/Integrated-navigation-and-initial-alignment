function [qbn,qbb]=AttitudeUpdate_Binary_sample(qbn0,Deltatheta0,Deltatheta)
    %��̬��Ԫ������
    %���루����Ԫ��̬��Ԫ��������Ԫ�������������Ԫ���������
    %���������Ԫ��̬��Ԫ��)
    %�޵������������������㷨��
    
    Phi = Deltatheta+Deltatheta0+2/3*cross(Deltatheta0,Deltatheta);
    qbb=[cos(0.5*norm(Phi));sin(0.5*norm(Phi))/(0.5*norm(Phi))*0.5*Phi];
 
    qbn=quatmul(qbn0,qbb);%��̬��Ԫ���˷�

    qbnValue=sqrt(qbn(1)*qbn(1)+qbn(2)*qbn(2)+qbn(3)*qbn(3)+qbn(4)*qbn(4));

    for i=1:4
        qbn(i)=qbn(i)/qbnValue;%��һ��
    end
end