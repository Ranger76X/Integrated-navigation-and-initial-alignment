function GPS_data = Interpolation_GPS(Ballistic_data,Ballistic_data_true,GPS_Hz,Ballistic_Hz,D_N)
    %生成模拟GPS数据
    %输入（模拟弹道数据,GPS更新频率）
    %输出（模拟GPS数据）
    
    step = Ballistic_Hz / GPS_Hz;
    GPS_data = [0,0,0];
    for i = 1:D_N
        if(mod((i-1),step))
            GPS_data(i,:) = Ballistic_data(i,:);
        else
            GPS_data(i,:) = Ballistic_data_true(i,:);
        end
    end
%     mvnrnd(0,0.01,D_N);




end