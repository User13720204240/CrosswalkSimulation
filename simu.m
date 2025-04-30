clear; clc;

%% Step 1: 输入参数
N_lanes = 2;
xc = 300;                       % 人行横道位置[m]
D = 3.5 * N_lanes;              % 人行横道宽度[m]
M = 0;                          % 中央安全岛 (1=有)
E_p = 0;                        % 电子警察 (1=有)
analysis_area = [0, 500];       % 仿真区域

% 仿真参数
dt = 0.1;                       % 时间步长[s]
T = 7200;                       % 总仿真时间[s]

% 流量参数
vehicle_rate = 300;             % [veh/h/lane]
pedestrian_rate = 250;          % [ped/h]

% Car-following模型参数
kappa = 0.41; lambda = 0.2;
V1 = 6.75; V2 = 7.91; C1 = 0.13; C2 = 1.57;
Lc = 9;                         % 车辆长度[m]
V_max = V1 + V2;                % 最大速度限制

% 行人参数
v_ped_range = [0.5, 1.5];       % 行人速度范围

% 预分配矩阵
max_vehicles = ceil(vehicle_rate * N_lanes * T / 3600);
max_pedestrians = ceil(pedestrian_rate * T / 3600);
% 车辆矩阵: [id, lane, x, v, yield, entry_time, exit_time]
done_vehicles = nan(max_vehicles, 7);
done_veh_count = 0;
% 行人矩阵: [id, position, wait_time, cross_time, status, v]
done_pedestrians = nan(max_pedestrians, 6);
done_ped_count = 0;

%% 初始化
t = 0;
vehicle_id = 0; ped_id = 0;
vehicles = nan(max_vehicles, 7);
pedestrians = nan(max_pedestrians, 6);
vehicle_arrivals = poissrnd((vehicle_rate/3600)*dt, [N_lanes, ceil(T/dt)]);
pedestrian_arrivals = poissrnd((pedestrian_rate/3600)*dt, [1, ceil(T/dt)]);

waiting_peds = zeros(1, ceil(T/dt));
crossing_peds = zeros(1, ceil(T/dt));
total_waiting_time = 0;
total_crossing_time = 0;

%% 主仿真循环
num_steps = ceil(T / dt);
for step = 1:num_steps
    t = (step - 1) * dt;
    t_index = step;

    %% Step 2: 生成车辆和行人
    for lane = 1:N_lanes
        if vehicle_arrivals(lane, t_index)
            vehicle_id = vehicle_id + 1;
            vehicles(vehicle_id, :) = [vehicle_id, lane, analysis_area(1), V1, 0, t, NaN];
            fprintf('新车辆进入: ID=%d, 时间=%.1f s\n', vehicle_id, t);
        end
    end

    if pedestrian_arrivals(1, t_index)
        ped_id = ped_id + 1;
        pedestrians(ped_id, :) = [ped_id, 1, 0, 0, 0, v_ped_range(1)+rand*(v_ped_range(2)-v_ped_range(1))];
        fprintf('新行人进入: ID=%d, 时间=%.1f s\n', ped_id, t);
    end

    %% Step 3: 统计行人状态
    active_peds = ~isnan(pedestrians(:, 1));
    W_t = sum(pedestrians(active_peds, 5) == 0);
    C_t = sum(pedestrians(active_peds, 5) == 1);
    waiting_peds(t_index) = W_t;
    crossing_peds(t_index) = C_t;

    %% Step 4: 行人决策与更新
    for p = 1:size(pedestrians, 1)
        if ~isnan(pedestrians(p, 1)) && pedestrians(p, 5) == 0 % waiting
            accept = ones(1, N_lanes);
            for lane = 1:N_lanes
                G_l = Inf;
                for v = 1:size(vehicles, 1)
                    if ~isnan(vehicles(v, 1)) && vehicles(v, 2) == lane && vehicles(v, 3) < xc
                        dist = xc - vehicles(v, 3);
                        if vehicles(v, 4) > 0
                            temp_G = dist / vehicles(v, 4);
                            G_l = min(G_l, temp_G);
                        end
                    end
                end
                U_il = 6.365 + 2.678*G_l - 2.846*D + 0.058*pedestrians(p, 3) - 1.273*pedestrians(p, 2);
                p_il = 1 / (1 + exp(-U_il));
                accept(lane) = (p_il >= 0.5);
            end
            if all(accept)
                pedestrians(p, 5) = 1; % crossing
                fprintf('行人 %d 开始过街, 时间=%.1f s\n', pedestrians(p, 1), t);
            end
        end

        if ~isnan(pedestrians(p, 1))
            if pedestrians(p, 5) == 1
                pedestrians(p, 4) = pedestrians(p, 4) + dt;
                if pedestrians(p, 4) >= D / pedestrians(p, 6)
                    pedestrians(p, 5) = 2; % finished
                    fprintf('行人 %d 完成过街, 时间=%.1f s, 等待时间=%.1f s\n', ...
                        pedestrians(p, 1), t, pedestrians(p, 3));
                end
            elseif pedestrians(p, 5) == 0
                pedestrians(p, 3) = pedestrians(p, 3) + dt;
            end
        end
    end

    %% 累积时间统计
    total_waiting_time = total_waiting_time + W_t*dt;
    total_crossing_time = total_crossing_time + C_t*dt;

    %% Step 5&6: 车辆让行判定与FVD跟驰
    active_vehs = ~isnan(vehicles(:, 1));
    if any(active_vehs)
        for v = 1:size(vehicles, 1)
            if ~isnan(vehicles(v, 1)) && isnan(vehicles(v, 7))
                % 查找前车
                dx = Inf; dv = 0; front_pass = true;
                for vp = 1:size(vehicles, 1)
                    if vp == v || isnan(vehicles(vp, 1))
                        continue;
                    end
                    if vehicles(vp, 2) == vehicles(v, 2) && vehicles(vp, 3) > vehicles(v, 3)
                        d_temp = vehicles(vp, 3) - vehicles(v, 3);
                        if d_temp < dx
                            dx = d_temp;
                            dv = vehicles(vp, 4) - vehicles(v, 4);
                            front_pass = false;
                        end
                    end
                end

                if dx < Inf && (vehicles(v, 3) + dx) >= xc
                    front_pass = true;
                end

                if vehicles(v, 3) < xc && front_pass
                    if C_t > 0
                        vehicles(v, 5) = 1;
                    elseif W_t > 0
                        dist = xc - vehicles(v, 3);
                        G_n = dist/max(vehicles(v, 4), 0.1);
                        Z_n = 1.291 + 2.937*G_n - 8.462*N_lanes + 0.730*W_t + 3.008*M + 11.913*E_p;
                        vehicles(v, 5) = (1/(1+exp(-Z_n))>=0.5);
                    else
                        vehicles(v, 5) = 0;
                    end
                else
                    vehicles(v, 5) = 0;
                end

                if vehicles(v, 3) >= xc
                    Vopt = V1 + V2*tanh(C1*(dx-Lc)-C2);
                    acc = kappa*(Vopt-vehicles(v, 4));
                elseif vehicles(v, 5) == 1
                    dn = xc - vehicles(v, 3);
                    Vpy = max(0, V1 + V2*tanh(C1*(dn-Lc)-C2));
                    acc = kappa*(Vpy-vehicles(v, 4)) - lambda*vehicles(v, 4);
                else
                    Vopt = V1 + V2*tanh(C1*(dx-Lc)-C2);
                    acc = kappa*(Vopt-vehicles(v, 4)) + lambda*dv;
                end
                vehicles(v, 4) = max(0, min(V_max, vehicles(v, 4) + acc*dt));
                vehicles(v, 3) = vehicles(v, 3) + vehicles(v, 4)*dt + 0.5*acc*dt^2;
                if vehicles(v, 3) >= analysis_area(2)
                    vehicles(v, 7) = t;
                end
            end
        end

        done_ped_idx = find(pedestrians(:, 5) == 2);
        for idx = done_ped_idx'
            done_ped_count = done_ped_count + 1;
            done_pedestrians(done_ped_count, :) = pedestrians(idx, :);
            pedestrians(idx, :) = nan(1, 6);
        end

        done_veh_idx = find(~isnan(vehicles(:, 7)));
        for idx = done_veh_idx'
            done_veh_count = done_veh_count + 1;
            done_vehicles(done_veh_count, :) = vehicles(idx, :);
            fprintf('已完成车辆: %d\n', done_vehicles(done_veh_count, 1));
            vehicles(idx, :) = nan(1, 7);
        end
    end
end

%% Step 8: 基于已完成实体统计输出指标
if done_veh_count > 0
    free_time = (analysis_area(2)-analysis_area(1)) / V_max;
    veh_delays = done_vehicles(1:done_veh_count, 7) - done_vehicles(1:done_veh_count, 6) - free_time;
    Dveh = mean(veh_delays);
else
    Dveh = NaN;
end

if done_ped_count > 0
    ped_waits = done_pedestrians(1:done_ped_count, 3);
    Wped = mean(ped_waits);
else
    Wped = NaN;
end

fprintf('===== 仿真结果 =====\n');
fprintf('车辆平均延误 Dveh = %.2f s\n', Dveh);
fprintf('行人平均等待时间 Wped = %.2f s\n', Wped);
fprintf('总行人数量: %d\n', done_ped_count);
fprintf('总车辆数量: %d\n', done_veh_count);