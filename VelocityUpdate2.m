function [v]=VelocityUpdate2(v0,XYZ,Deltat,Cbn0,Deltav,WGS84)
    %�ٶȸ��º���(�����أ�
    %���루����Ԫ�ٶ�(�����أ�������Ԫλ�á���Ԫʱ����������Ԫ��̬���󡢱���Ԫ���ٶȼ����)
    %���������Ԫ�ٶȣ�
    format long
    
    gl = [0;0;WGS84.ge*(1-2*XYZ(3)/WGS84.R)];

    DeltaVgk=(gl)*Deltat;

    DeltaVfn=Cbn0*Deltav;

    v=v0-DeltaVgk+DeltaVfn;
end