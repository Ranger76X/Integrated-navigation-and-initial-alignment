function [f] = f2(x,N,Alpha,Beta)

%%%%%%%%%%f(x)%%%%%%%%%%%
    A  = 0;
    x = x/180*pi;
    Cnb = q2mat(Euler2Quaternion(x(1),(75)/180*pi,x(2)));
    
    for i = 1:N
        A = A + (Beta(:,i) - Cnb*Alpha(:,i))'*(Beta(:,i) - Cnb*Alpha(:,i));   
    end
    f = 1/(2*N) * (A);
%%%%%%%%%%f(x)%%%%%%%%%%%

end