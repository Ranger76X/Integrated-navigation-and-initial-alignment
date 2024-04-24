弹道仿真/初始对准/组合导航程序

test_function_bas3_3_6可以正确解算炮弹的初始三轴姿态。(发射前不装订俯仰角，偏航角方案。二次对准)

test_function_basroll可以正确解算炮弹的初始横滚角姿态。(发射前装订俯仰角，偏航角方案。单次对准)

test_function_bas7为100次弹道三轴姿态初始对准仿真实验程序。

test_function_bas7w为100次弹道弹剑横滚角单轴初始对准仿真实验程序。

test_function_SVD为基于奇异值分解的初始对准方法。

INS3为弹载惯性导航程序，其他INS程序存在错误。其中INS2中传感器数据为炮弹实射数据。传感器数据中存在较多错误测量值需要处理，直接使用结果错误。

test_function_EKF/4/2为弹载INS/GNSS组合导航程序。
其中test_function_EKF为原始卡尔曼滤波组合导航方法。
test_function_EKF4为基于HUBER权值的卡尔曼滤波组合导航方法。
test_function_EKF2存在问题效果差。

f_yaw/test/roll/pitch/2/3_2/3_3/3/4/5/6以及f和fun为测试用的wahba代价函数及其改进函数。
其中三轴初始对准程序最终采用的代价函数为f3_2。
f_yaw/roll/pitch为单轴初始对准wahba代价函数。

three_axie_fun以及Untitled和Untitled2以及result文件夹中Untitled2用于计算代价函数函数数值分布，计算量较大，非必要不要运行！
result文件夹中Untitled可以运行，从二维视角展示寻优过程。
result文件夹中的7、文件记录测试（三维图）文件夹内程序展示三轴代价函数数值的三维分布图。

文件夹中的各类VelocityUpdate以及AttitudeUpdate效果参考日志文件夹中内容以及INS_SIM中的readme文件。