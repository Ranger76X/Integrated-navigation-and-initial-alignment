function GPS_data = Generate_GPS(Ballistic_data,GPS_Hz,Ballistic_Hz,D_N)
    %生成模拟GPS数据
    %输入（模拟弹道数据,GPS更新频率）
    %输出（模拟GPS数据）
    
    step = Ballistic_Hz / GPS_Hz;
    GPS_data = [0,0,0];
    for i = 1:D_N
        if(mod((i-1),step))
            GPS_data(i,:) = Ballistic_data(i,:);
        else
            GPS_data(i,:) = [0,0,0];
        end
    end
%     mvnrnd(0,0.01,D_N);




end