function [XYZ]=LocationUpdata(XYZ0,v,v_pre,Deltat)
    %λ�ø���
    %���루�ϸ���Ԫ��λ�á�����Ԫ�ٶȡ�����ʱ������
    %�������λ�ã�XYZ��
    
    XYZ(1) = XYZ0(1) + (v(1)+v_pre(1))/2 * Deltat;
    XYZ(3) = XYZ0(3) + (v(3)+v_pre(3))/2 * Deltat;
%     *R/(R+yn(2));
    XYZ(2) = XYZ0(2) + (v(2)+v_pre(2))/2 * Deltat;
     
end