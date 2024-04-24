function [f] = fun(x,N,Alpha,Beta)

 %%%%%%%%%%f(x)%%%%%%%%%%%
    A  = 0;
    x = x/180*pi;
    Cnb = q2mat(Euler2Quaternion(45/180*pi,0,x))';
    
    for i = 1:N
        A = A + (Alpha(:,i) - Cnb*Beta(:,i))'*(Alpha(:,i) - Cnb*Beta(:,i));   
    end
    f = 1/(2*N) * (A);
%%%%%%%%%%f(x)%%%%%%%%%%%



end