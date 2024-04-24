function [f] = f(x,N,Alpha,Beta,noise1,noise2)
% function [f] = f(x,N,Alpha,Beta)
%
 %%%%%%%%%%f(x)%%%%%%%%%%%
    A  = 0;
    x = x/180*pi;
    Cnb = q2mat(Euler2Quaternion((45+noise1)/180*pi,x,(-30+noise2)/180*pi))';
%     Cnb = q2mat(Euler2Quaternion((45)/180*pi,x,(-30)/180*pi))';
    
    for i = 1:N
        A = A + (Alpha(:,i) - Cnb*Beta(:,i))'*(Alpha(:,i) - Cnb*Beta(:,i));   
    end
    f = (1/(2*N) * (A));
%%%%%%%%%%f(x)%%%%%%%%%%%



end