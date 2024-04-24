

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%BAS初始化部�?1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = 120;%�?要的观测矢量对数

BAS.eta = 0.95;
BAS.c = 5;                  %ratio between step and d0
BAS.step = 4;               %initial step set as the largest input range
BAS.n = 1000;                %iterations
BAS.k = 1;                  %space dimension
BAS.x = mod(0,360);        %intial value
BAS.xbest = BAS.x;
BAS.f = f(BAS.x,N,vb,Beta);
BAS.fbest = BAS.f;
BAS.fbest_store = BAS.fbest;
BAS.x_store = [0;BAS.x;BAS.fbest];


for i = 1:BAS.n
    BAS.d0=BAS.step/BAS.c;
    BAS.dir=rands(BAS.k,1);
    BAS.dir=BAS.dir/(eps+norm(BAS.dir));    
    
    BAS.xleft=BAS.x+BAS.dir*BAS.d0;
    BAS.fleft=f(BAS.xleft,N,vb,Beta);
    BAS.xright=BAS.x-BAS.dir*BAS.d0;
    BAS.fright=f(BAS.xright,N,vb,Beta);    
    
    BAS.x=BAS.x-BAS.step*BAS.dir*sign(BAS.fleft-BAS.fright);
    
    BAS.f = f(BAS.x,N,vb,Beta);
    
    %%%%%%%%%%
    if BAS.f<BAS.fbest
        BAS.xbest=BAS.x;
        BAS.fbest=BAS.f;
    end
    %%%%%%%%%%%
    BAS.x_store=cat(2,BAS.x_store,[i;BAS.x;BAS.f]);
    BAS.fbest_store=[BAS.fbest_store;BAS.fbest];
    phi(i) = BAS.xbest;
    display([num2str(i),':xbest=[',num2str(BAS.xbest'),'],fbest=',num2str(BAS.fbest)])
    %%%%%%%%%%%
%     if(BAS.fbest > 0.04)
%         BAS.step=BAS.step;
%     elseif(BAS.fbest < 0.04 && BAS.fbest > 0.005)
%         BAS.step=BAS.step*0.99;
%     else
        BAS.step=BAS.step*BAS.eta;
%     end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%数据显示部分
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1);
clf(1);
plot(BAS.x_store(1,:),BAS.x_store(end,:),'r-o');
hold on;
plot(BAS.x_store(1,:),BAS.fbest_store,'b-.');
xlabel('iteration');
ylabel('minimum value');
figure(2);
clf(2);
plot(BAS.x_store(2,:));
hold on;
plot(Euler(2,:));
