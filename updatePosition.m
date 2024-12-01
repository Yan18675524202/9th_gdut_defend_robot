
function [new_position, new_velocity] = updatePosition(current_position, current_velocity, acceleration, max_speed, dt)
    % current_position: 当前坐标 (1x2 向量) [x, y]
    % current_velocity: 当前速度 (1x2 向量) [vx, vy]
    % acceleration: 当前加速度 (1x2 向量) [ax, ay]
    % max_speed: 最大速度限制 (标量)
    % dt: 时间步长 (标量)

    % 计算下一个时间单位的速度
    new_velocity = current_velocity + acceleration * dt;

    % 计算速度的模长
    speed_magnitude = norm(new_velocity);

    % 检查速度是否超过最大速度限制
    if speed_magnitude > max_speed
        % 缩放速度
        scaling_factor = max_speed / speed_magnitude;
        new_velocity = new_velocity * scaling_factor;
    end

    % 更新位置
    new_position = current_position + new_velocity * dt;

end