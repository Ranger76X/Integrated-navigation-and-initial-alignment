function [qbn,qbb]=AttitudeUpdate3(qbn0,Deltatheta0,Deltatheta1,Deltatheta2,Deltatheta3)
    %��̬��Ԫ������
    %���루����Ԫ��̬��Ԫ��������Ԫ�������������Ԫ���������
    %���������Ԫ��̬��Ԫ����
    %�޵������������������㷨
    Phi = Deltatheta0 + 9/20*cross(Deltatheta1,Deltatheta3) + 27/40*cross(Deltatheta2,(Deltatheta3-Deltatheta1));
%     Phi = Deltatheta0+2/3*cross(Deltatheta1,Deltatheta2);
    qbb=[cos(0.5*norm(Phi));sin(0.5*norm(Phi))/(0.5*norm(Phi))*0.5*Phi];
 
    qbn=quatmul(qbn0,qbb);%��̬��Ԫ���˷�

    qbnValue=sqrt(qbn(1)*qbn(1)+qbn(2)*qbn(2)+qbn(3)*qbn(3)+qbn(4)*qbn(4));

    for i=1:4
        qbn(i)=qbn(i)/qbnValue;%��һ��
    end
end