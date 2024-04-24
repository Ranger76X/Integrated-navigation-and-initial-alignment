function Cnb = Quaternion2ACM(qnb)
% 四元数转旋转矩阵
% 东北天坐标系，旋转顺序Z，X，Y（312）

% Convert attitude quaternion to direction cosine matrix(DCM).
%
% Prototype: Cnb = q2mat(qnb)
% Input: qnb - attitude quaternion
% Output: Cnb - DCM from body-frame to navigation-frame
    q11 = qnb(1)*qnb(1); q12 = qnb(1)*qnb(2); q13 = qnb(1)*qnb(3); q14 = qnb(1)*qnb(4); 
    q22 = qnb(2)*qnb(2); q23 = qnb(2)*qnb(3); q24 = qnb(2)*qnb(4);     
    q33 = qnb(3)*qnb(3); q34 = qnb(3)*qnb(4);  
    q44 = qnb(4)*qnb(4);
    Cnb = [ q11+q22-q33-q44,  2*(q23-q14),     2*(q24+q13);
            2*(q23+q14),      q11-q22+q33-q44, 2*(q34-q12);
            2*(q24-q13),      2*(q34+q12),     q11-q22-q33+q44 ];
end