function [v]=VelocityUpdate3(v0,Deltat,Cbn0,Cbn0_pre,acc,acc_pre)
    %�ٶȸ��º���(�����أ�
    %���루����Ԫ�ٶ�(�����أ�������Ԫλ�á���Ԫʱ����������Ԫ��̬���󡢱���Ԫ���ٶȼ����)
    %���������Ԫ�ٶȣ�
    format long
    

%     gl = [0;0;WGS84.ge*(1-2*XYZ(3)/WGS84.R)];
    gl = [0;0;9.8];
%     acc = Cbn0*acc+gl/1.55;
%     acc_pre = Cbn0_pre*acc_pre+gl/1.55;
  
    acc = Cbn0*acc+gl;
    acc_pre = Cbn0_pre*acc_pre+gl;
%     DeltaVgk=(gl)*Deltat;

    DeltaVfn=(acc_pre+acc)/2*Deltat;

    v=v0+DeltaVfn;
end