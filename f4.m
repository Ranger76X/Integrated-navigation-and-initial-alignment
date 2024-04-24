function [f] = f4(x,N1,N2,Alpha,Beta,pitch,yaw)
%
 %%%%%%%%%%f(x)%%%%%%%%%%%
    A  = 0;
    x = x/180*pi;
    Cnb = q2mat(Euler2Quaternion((pitch)/180*pi,x,(yaw)/180*pi))';
    
    for i = N1:N1+N2
        A = A + (Alpha(:,i) - Cnb*Beta(:,i))'*(Alpha(:,i) - Cnb*Beta(:,i));   
    end
    f = (1/(2*N2) * (A))^(1/4);
%%%%%%%%%%f(x)%%%%%%%%%%%



end