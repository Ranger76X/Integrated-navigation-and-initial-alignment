function [XYZ]=LocationUpdata2(XYZ0,v,Deltat)
    %λ�ø���
    %���루�ϸ���Ԫ��λ�á�����Ԫ�ٶȡ�����ʱ������
    %�������λ�ã�XYZ��
    
    XYZ(1) = XYZ0(1) - v(1) * Deltat;
%     *R/(R+yn(2));
    XYZ(2) = XYZ0(2) + v(2) * Deltat;
    XYZ(3) = XYZ0(3) - v(3) * Deltat;  
end