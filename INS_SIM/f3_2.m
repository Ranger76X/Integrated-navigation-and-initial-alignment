function [f] = f3_2(x,N,Alpha,Beta,start)

 %%%%%%%%%%f(x)%%%%%%%%%%%
    A  = 0;
%     x = [mod(x(1),90),mod(x(2),180),mod(x(3),180)];%%
    x = x/180*pi;
%     Cnb = q2mat(Euler2Quaternion(1/180*pi,1/180*pi,x(3)));
    Cnb = q2mat(Euler2Quaternion(x(1),x(2),x(3)));
    for i = start:start+N
        A = A + (Beta(:,i) - Cnb*Alpha(:,i))'*(Beta(:,i) - Cnb*Alpha(:,i));   
    end
    f = (1/(2*N) * (A))^(1/4);
%%%%%%%%%%f(x)%%%%%%%%%%%



end