% -------------------------------------------------------------------------
%
% File : DynamicWindowApproachSample.m
%
% Discription : Mobile Robot Motion Planning with Dynamic Window Approach
%
% Environment : Matlab

% -------------------------------------------------------------------------





function [] = run()

close all;
clear all;

disp('Dynamic Window Approach sample program start!!')

%% 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
% x=[0 0 pi/2 0 0]'; % 5x1矩阵 列矩阵  位置 0，0 航向 pi/2 ,速度、角速度均为0
robot_1 = [1 2 pi/2 0 0]'; 
robot_2 = [5 2 pi/2 0 0]';

% 下标宏定义 状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
POSE_X      = 1;  %坐标 X
POSE_Y      = 2;  %坐标 Y
YAW_ANGLE   = 3;  %机器人航向角
V_SPD       = 4;  %机器人速度
W_ANGLE_SPD = 5;  %机器人角速度 

goal = [3,13.5];   % 目标点位置 [x(m),y(m)]

% 防守机器人位置列表 [x(m) y(m)]
robot_A=[1 13];
robot_B=[6 13];


obstacleR = 0.5;% 冲突判定用的障碍物半径
global dt; 
dt = 0.1;% 时间[s]

% 机器人运动学模型参数
% 最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],
% 速度分辨率[m/s],转速分辨率[rad/s]]
Kinematic_1 = [2.0,toRadian(60.0),1.0,toRadian(100.0),0.05,toRadian(5)];
Kinematic_2 = [2.0,toRadian(60.0),1.0,toRadian(100.0),0.05,toRadian(5)];

%定义Kinematic的下标含义
MD_MAX_V    = 1;%   最高速度m/s]
MD_MAX_W    = 2;%   最高旋转速度[rad/s]
MD_ACC      = 3;%   加速度[m/ss]
MD_VW       = 4;%   旋转加速度[rad/ss]
MD_V_RESOLUTION  = 5;%  速度分辨率[m/s]
MD_W_RESOLUTION  = 6;%  转速分辨率[rad/s]]


% 评价函数参数 [heading,dist,velocity,predictDT]
% 航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
evalParam_1 = [0.01, 1.4 ,0.6, 1.0];
evalParam_2 = [0.01, 1.4 ,0.6, 1.0];

area      = [-3 9 -3 15];% 模拟区域范围 [xmin xmax ymin ymax]
area1      = [-1 7 -1 14];
previous_error = [0 0 0 0 0 0 0 0];
integral = [0 0 0 0 0 0 0 0];
target_v1 = 0;
target_v2 = 0;


robot_vA = [0 0];
robot_aA = [0 0];

robot_vB = [0 0];
robot_aB = [0 0];

targetx1 = 0;
targety1 = 0;

targetx2 = 0;
targety2 = 0;
% Vx = 0.0;      % 当前 x 方向速度
% Vy = 0.0;      % 当前 y 方向速度
% ax = 0.0;      % x 方向加速度
% ay = 0.0;      % y 方向加速度
Vmax = 2;

% 模拟实验的结果
result.x1=[];   %累积存储走过的轨迹点的状态值 
result.x2=[];   %累积存储走过的轨迹点的状态值 


tic; % 估算程序运行时间开始

% movcount=0;
%% Main loop   循环运行 5000次 指导达到目的地 或者 5000次运行结束
for i = 1:5000  


% 防守策略
% 计算机器人与目标点的距离
D_1 = sqrt((robot_1(POSE_X) - goal(POSE_X))^2 + (robot_1(POSE_Y) - goal(POSE_Y))^2);
D_2 = sqrt((robot_2(POSE_X) - goal(POSE_X))^2 + (robot_2(POSE_Y) - goal(POSE_Y))^2);
% 防守车之间的距离
D_AtoB = sqrt((robot_A(1,1) - robot_B(1,1))^2 + (robot_A(1,2) - robot_B(1,2))^2);

D_Ato1 = sqrt((robot_A(1,1) - robot_1(POSE_X))^2 + (robot_A(1,2) - robot_1(POSE_Y))^2);
D_Ato2 = sqrt((robot_A(1,1) - robot_2(POSE_X))^2 + (robot_A(1,2) - robot_2(POSE_Y))^2);
D_Bto1 = sqrt((robot_B(1,1) - robot_1(POSE_X))^2 + (robot_B(1,2) - robot_1(POSE_Y))^2);
D_Bto2 = sqrt((robot_B(1,1) - robot_2(POSE_X))^2 + (robot_B(1,2) - robot_2(POSE_Y))^2);
% 安全距离阈值
safe_distance1 = 1.2;
safe_distance2 = 1.6;

sign_1 = 0;
sign_2 = 0;

% 判断机器人与目标点的距离
if D_1 >= 8.0 && D_2 >= 8.0
    targetx1 = 2;
    targety1 = 8;


else 
   
    if D_2 < 8.0 && D_1 < 8.0

        if D_Ato1 < D_Bto1
            sign_1 = 1;
            if(D_1  < 6 )
      
                targetx1 = robot_1(POSE_X)  ; 
                targety1 = robot_1(POSE_Y)  + 0.5*(goal(POSE_Y) - robot_1(POSE_Y))/D_1;%逼近
            else
                targetx1 = robot_1(POSE_X)  ; 
                targety1 = robot_1(POSE_Y)  + 1.5*(goal(POSE_Y) - robot_1(POSE_Y))/D_1;
            end  

        else
            sign_2 = 1;
            if(D_2  < 6 )
          
                targetx1 = robot_2(POSE_X)  ; 
                targety1 = robot_2(POSE_Y)  + 0.5*(goal(POSE_Y) - robot_2(POSE_Y))/D_2;%逼近
            else
                targetx1 = robot_2(POSE_X)  ; 
                targety1 = robot_2(POSE_Y)  + 1.5*(goal(POSE_Y) - robot_2(POSE_Y))/D_2;
            end  
        end
    end

    if D_2 < 8.0 && D_1 > 8.0
        if D_Ato2 < D_Bto2
            if(D_2  < 6 )
          
                targetx1 = robot_2(POSE_X)  ; 
                targety1 = robot_2(POSE_Y)  + 0.5*(goal(POSE_Y) - robot_2(POSE_Y))/D_2;%逼近
            else
                targetx1 = robot_2(POSE_X)  ; 
                targety1 = robot_2(POSE_Y)  + 1.5*(goal(POSE_Y) - robot_2(POSE_Y))/D_2;
            end  

            sign_2 = 1;
        end    


    end    
    
    if D_2 >= 8.0 && D_1 < 8.0
        if D_Ato1 < D_Bto1
            if(D_1  < 6 )
          
                targetx1 = robot_1(POSE_X)  ; 
                targety1 = robot_1(POSE_Y)  + 0.5*(goal(POSE_Y) - robot_1(POSE_Y))/D_1;%逼近
            else
                targetx1 = robot_1(POSE_X)  ; 
                targety1 = robot_1(POSE_Y)  + 1.5*(goal(POSE_Y) - robot_1(POSE_Y))/D_1;
            end  

            sign_1 = 1;
        end    


    end  

end
  
 

        %targetx1 = robot_2(POSE_X)  + 2*(goal(POSE_X) - robot_2(POSE_X))/D_2; % 障碍物在机器人和目标点之间2米处
        %targety1 = robot_2(POSE_Y)  + 2*(goal(POSE_Y) - robot_2(POSE_Y))/D_2;
   
    %disp(["x1=", num2str(targetx1)]);
    %disp(["y1=", num2str(targety1)]);
   

if  sign_1 == 1 && sign_2 == 0
        if  D_2  < 6 
             targetx2 = robot_2(POSE_X) ; 
             targety2 = robot_2(POSE_Y) + 0.5*(goal(POSE_Y) - robot_2(POSE_Y))/D_1;

        else
             targetx2 = robot_2(POSE_X) ; 
             targety2 = robot_2(POSE_Y) + 1.5*(goal(POSE_Y) - robot_2(POSE_Y))/D_1;
        end
         
       

end
if  sign_1 == 0 && sign_2 == 1
        if  D_1  < 6 
    
            targetx2 = robot_1(POSE_X) ; 
            targety2 = robot_1(POSE_Y) + 0.5*(goal(POSE_Y) - robot_1(POSE_Y))/D_2;

        else

            targetx2 = robot_1(POSE_X) ; 
            targety2 = robot_1(POSE_Y) + 1.5*(goal(POSE_Y) - robot_1(POSE_Y))/D_2;


        end

       
end
if  sign_1 == 0 && sign_2 == 0
    targetx2 = 4.5;
    targety2 = 8;

end

%if  D_2 >= 8.0
    % 机器人距离目标点8米以外时，障碍物在目标点三米内移动
    %targetx2 = 5;
    %targety2 = 8;

%else
    % 机器人进入目标点5米以内时，障碍物移动到机器人和目标点之间的连线上阻止机器人移动
   
        %targetx2 = robot_1(POSE_X) + 2*(goal(POSE_X) - robot_1(POSE_X))/D_1; % 障碍物在机器人和目标点之间2米处
        %targety2 = robot_1(POSE_Y) + 2*(goal(POSE_Y) - robot_1(POSE_Y))/D_1;
    
        %targetx2 = robot_2(POSE_X) + 1.5*(goal(POSE_X) - robot_2(POSE_X))/D_2; % 障碍物在机器人和目标点之间2米处
        %targety2 = robot_2(POSE_Y) + 1.5*(goal(POSE_Y) - robot_2(POSE_Y))/D_2;

      
   
    
%end
   

    [targetx1,targety1] = CalDist(targetx1,targety1,robot_B(1,1),robot_B(1,2),safe_distance2);
    [targetx1,targety1] = CalDist(targetx1,targety1,robot_1(1),robot_1(2),safe_distance1);
    [targetx1,targety1] = CalDist(targetx1,targety1,robot_2(1),robot_2(2),safe_distance1);
   
  
    [targetx2,targety2] = CalDist(targetx2,targety2,robot_A(1,1),robot_A(1,2),safe_distance2);
    [targetx2,targety2] = CalDist(targetx2,targety2,robot_1(1),robot_1(2),safe_distance1);
    [targetx2,targety2] = CalDist(targetx2,targety2,robot_2(1),robot_2(2),safe_distance1);


    


    [target_v1,previous_error(1),integral(1)] = PID(targetx1,robot_A(1,1),1.5,0.2,5.0,0,previous_error(1),integral(1));                 %防守姿态
    [robot_aA(1),previous_error(2),integral(2)] = PID(target_v1,robot_vA(1),1.5,0.1,5.0,1.0,previous_error(2),integral(2));



    [target_v2,previous_error(5),integral(5)] = PID(targetx2,robot_B(1,1),1.5,0.2,5.0,0,previous_error(5),integral(5));                 %防守姿态
    [robot_aB(1),previous_error(6),integral(6)] = PID(target_v2,robot_vB(1),1.5,0.1,5.0,1.0,previous_error(6),integral(6));
   

    [target_v1, previous_error(3), integral(3)] = PID(targety1, robot_A(1,2), 3.0, 0.01, 50.0, 0, previous_error(3), integral(3));
    [robot_aA(2), previous_error(4), integral(4)] = PID(target_v1, robot_vA(2), 3.0, 0.0, 2.0, 1.0, previous_error(4), integral(4));
    %disp(["ay1=", num2str(robot_aA(2))]);



    [target_v2, previous_error(7), integral(7)] = PID(targety2, robot_B(1,2), 3.0, 0.01, 50.0, 0, previous_error(7), integral(7));
    [robot_aB(2), previous_error(8), integral(8)] = PID(target_v2, robot_vB(2), 3.0, 0.0, 2.0, 1.0, previous_error(8), integral(8));
    %disp(["ay1=", num2str(robot_aB(2))]);


% 更新障碍物的位置和速度
[robot_A, robot_vA] = updatePosition(robot_A, robot_vA, robot_aA, Vmax, dt);
%disp(["Vy=", num2str(robot_vA(2))]);
[robot_B, robot_vB] = updatePosition(robot_B, robot_vB, robot_aB, Vmax, dt);
%disp(["Vy=", num2str(robot_vB(2))]);


pos1 = [robot_1(1),robot_1(2)];
pos2 = [robot_2(1),robot_2(2)];
pos3 = [robot_A(1,1),robot_A(1,2)];
pos4 = [robot_B(1,1),robot_B(1,2)];
% DWA参数输入 返回控制量 u = [v(m/s),w(rad/s)] 和 轨迹

[u1,traj1] = DynamicWindowApproach(robot_1,Kinematic_1,goal,evalParam_1,pos2,pos3,pos4,obstacleR,area1);
[u2,traj2] = DynamicWindowApproach(robot_2,Kinematic_2,goal,evalParam_2,pos1,pos3,pos4,obstacleR,area1);

robot_2 = f(robot_2,u2);% 机器人移动到下一个时刻的状态量 根据当前速度和角速度推导 下一刻的位置和角度
robot_1 = f(robot_1,u1);% 机器人移动到下一个时刻的状态量 根据当前速度和角速度推导 下一刻的位置和角度

% 历史轨迹的保存
result.x1 = [result.x1; robot_1'];  %最新结果 以列的形式 添加到result.x1
result.x2 = [result.x2; robot_2'];  %最新结果 以列的形式 添加到result.x2

% 是否到达目的地
if norm(robot_1(POSE_X:POSE_Y)-goal')<0.5   % norm函数来求得坐标上的两个点之间的距离
disp('Arrive Goal!!');break;
end

if norm(robot_2(POSE_X:POSE_Y)-goal')<0.5   % norm函数来求得坐标上的两个点之间的距离
disp('Robot 2 Arrive Goal!!');break;
end

%====Animation====
hold off;               % 关闭图形保持功能。 新图出现时，取消原图的显示。


ArrowLength = 0.3;      % 箭头长度

% 机器人
% quiver(x,y,u,v) 在 x 和 y 中每个对应元素对组所指定的坐标处将向量绘制为箭头
quiver(robot_1(POSE_X), robot_1(POSE_Y), ArrowLength*cos(robot_1(YAW_ANGLE)), ArrowLength*sin(robot_1(YAW_ANGLE)), 'ok'); % 绘制机器人当前位置的航向箭头

quiver(robot_2(POSE_X), robot_2(POSE_Y), ArrowLength*cos(robot_2(YAW_ANGLE)), ArrowLength*sin(robot_2(YAW_ANGLE)), 'or'); % 绘制机器人当前位置的航向箭头


rectangle('Position', [-1, -1, 8, 15], 'EdgeColor', 'k', 'LineWidth', 2);  
hold on;                                                     %启动图形保持功能，当前坐标轴和图形都将保持，从此绘制的图形都将添加在这个图形的基础上，并自动调整坐标轴的范围

plot(result.x1(:,POSE_X),result.x1(:,POSE_Y),'-b');hold on;    % 绘制走过的所有位置 所有历史数据的 X、Y坐标
plot(result.x2(:,POSE_X),result.x2(:,POSE_Y),'-r');hold on;    % 绘制走过的所有位置 所有历史数据的 X、Y坐标

plot(goal(1),goal(2),'*r');hold on;                          % 绘制目标位置
% 绘制机器人和目标点之间的连线
plot([robot_1(1), goal(1)], [robot_1(2), goal(2)], 'k--'); % 黑色虚线
plot([robot_2(1), goal(1)], [robot_2(2), goal(2)], 'r--'); % 红色虚线
plot([-1, 7], [6,6], 'k'); % 黑色虚线
%plot(obstacle(:,1),obstacle(:,2),'*k');hold on;              % 绘制所有障碍物位置
DrawObstacle_plot(robot_A,obstacleR);
DrawObstacle_plot(robot_B,obstacleR);
DrawObstacle_plot4(robot_1,obstacleR);
DrawObstacle_plot4(robot_2,obstacleR);

% 探索轨迹 画出待评价的轨迹
if ~isempty(traj1) %轨迹非空
for it=1:length(traj1(:,1))/5    %计算所有轨迹数  traj 每5行数据 表示一条轨迹点
ind = 1+(it-1)*5; %第 it 条轨迹对应在traj中的下标 
plot(traj1(ind,:),traj1(ind+1,:),'-g');hold on;  %根据一条轨迹的点串画出轨迹   traj(ind,:) 表示第ind条轨迹的所有x坐标值  traj(ind+1,:)表示第ind条轨迹的所有y坐标值
end
end
if ~isempty(traj2) %轨迹非空
for it=1:length(traj2(:,1))/5    %计算所有轨迹数  traj 每5行数据 表示一条轨迹点
ind = 1+(it-1)*5; %第 it 条轨迹对应在traj中的下标 
plot(traj2(ind,:),traj2(ind+1,:),'-m');hold on;  %根据一条轨迹的点串画出轨迹   traj(ind,:) 表示第ind条轨迹的所有x坐标值  traj(ind+1,:)表示第ind条轨迹的所有y坐标值
end
end



axis(area); %根据area设置当前图形的坐标范围，分别为x轴的最小、最大值，y轴的最小最大值
grid on;
drawnow;  %刷新屏幕. 当代码执行时间长，需要反复执行plot时，Matlab程序不会马上把图像画到figure上，这时，要想实时看到图像的每一步变化情况，需要使用这个语句。
%movcount = movcount+1;
%mov(movcount) = getframe(gcf);%  记录动画帧
end
toc  %输出程序运行时间  形式：时间已过 ** 秒。
%movie2avi(mov,'movie.avi');  %录制过程动画 保存为 movie.avi 文件

%% 绘制所有障碍物位置
% 输入参数：obstacle 所有障碍物的坐标   obstacleR 障碍物的半径
function [] = DrawObstacle_plot(obstacle,obstacleR)
r = obstacleR; 
theta = 0:pi/20:2*pi;
for id=1:length(obstacle(:,1))
x = r * cos(theta) + obstacle(id,1); 
y = r  *sin(theta) + obstacle(id,2);
plot(x,y,'-m');hold on; 
end
function [] = DrawObstacle_plot4(xt,obstacleR)
r = obstacleR; 
theta = 0:pi/20:2*pi;
for id=1:length(xt(1,1))
 x = r * cos(theta) + xt(1); 
 y = r  *sin(theta) + xt(2);
 plot(x,y,'-g');hold on; 
end
%plot(obstacle(:,1),obstacle(:,2),'*m');hold on;              % 绘制所有障碍物位置



%% END

