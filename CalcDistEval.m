%% 障碍物距离评价函数  （机器人在当前轨迹上与最近的障碍物之间的距离，如果没有障碍物则设定一个常数）
% 输入参数：位姿、所有障碍物位置、障碍物半径
% 输出参数：当前预测的轨迹终点的位姿距离所有障碍物中最近的障碍物的距离 如果大于设定的最大值则等于最大值
% 距离障碍物距离越近分数越低
function dist = CalcDistEval(x,robot_A,robot_B,pos,R,area)
dist=100;
disttmp = x(1) - area(1)-1;
if dist > disttmp   % 大于最小值 则选择最小值
    dist = disttmp;
end

disttmp = area(2)-1-x(1);
if dist > disttmp   % 大于最小值 则选择最小值
    dist = disttmp;
end

disttmp = x(2) - area(3)+1;
if dist > disttmp   % 大于最小值 则选择最小值
    dist = disttmp;
end

disttmp = area(4)-1 - x(2);
if dist > disttmp   % 大于最小值 则选择最小值
    dist = disttmp;
end

for io = 1:length(robot_A(:,1))
    disttmp = norm(robot_A(io,:)-x(1:2)')-R; %到第io个障碍物的距离 - 障碍物半径  ！！！有可能出现负值吗？？
    if dist > disttmp   % 大于最小值 则选择最小值
        dist = disttmp;
    end
end

for io = 1:length(robot_B(:,1))
    disttmp = norm(robot_B(io,:)-x(1:2)')-R; %到第io个障碍物的距离 - 障碍物半径  ！！！有可能出现负值吗？？
    if dist > disttmp   % 大于最小值 则选择最小值
        dist = disttmp;
    end
end


for io = 1:length(pos(:,1))
    disttmp = norm(pos(io,:)-x(1:2)')-R; %到第io个障碍物的距离 - 障碍物半径  ！！！有可能出现负值吗？？
    if dist > disttmp   % 大于最小值 则选择最小值
        dist = disttmp;
    end
end
% 障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
if dist >= 2*R %最大分数限制
    dist = 2*R;
end
end