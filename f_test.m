function [f] = f_test(x,N,Alpha,Beta)

%%%%%%%%%%f(x)%%%%%%%%%%%
    A  = 0;
    x = x/180*pi;
    Cnb = q2mat(Euler2Quaternion(x(1),x(2),-x(3)));
    
    for i = 1:N
        A = A + (Beta(:,i) - Cnb*Alpha(:,i))'*(Beta(:,i) - Cnb*Alpha(:,i));   
    end
    f = 1/(2*N) * (A);
%%%%%%%%%%f(x)%%%%%%%%%%%

end