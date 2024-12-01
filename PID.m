function [control_signal,new_previous_error,new_integral] = PID(target, current,Kp,Ki,Kd,outmax,previous_error,integral)
    % 计算当前误差
    error = target - current;

    % 计算积分项
    new_integral = integral + error;
    if abs(new_integral) > 3
    new_integral = 3 * sign(new_integral);
    end

    % 计算微分项
    derivative = error - previous_error;

    % 计算控制信号
    control_signal = Kp * error + Ki * integral + Kd * derivative;

    if outmax ~= 0
        if abs(control_signal) > outmax
            if control_signal > 0
                control_signal = outmax;
            else
                control_signal = -outmax;
            end
        end
    end

    % 更新前一个误差
    new_previous_error = error;

end