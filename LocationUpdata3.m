function [XYZ]=LocationUpdata3(XYZ0,v,v_pre,Deltat,WGS84)
    %λ�ø���
    %���루�ϸ���Ԫ��λ�á�����Ԫ�ٶȡ�����ʱ������
    %�������λ�ã�XYZ��
    
    XYZ(1) = XYZ0(1) + (v(1)+v_pre(1))/2 * Deltat;
    XYZ(3) = XYZ0(3) + (v(3)+v_pre(3))/2 * Deltat;
%     *R/(R+yn(2));
    XYZ(2) = XYZ0(2) + (v(2)+v_pre(2))/2  * Deltat * WGS84.R/(WGS84.R+XYZ(3));
     
end