% 实现了一个无信号灯人行横道处车辆与行人交互的仿真
function simu_1()
% 创建主界面
fig = figure('Name','无信号街区人行横道人车交互仿真','NumberTitle','off','MenuBar','none','Position',[100,100,800,600]);

% 坐标轴
ax = axes('Parent',fig,'Units','normalized','Position',[0.05,0.25,0.9,0.65]);
hold(ax,'on'); grid(ax,'on'); ax.DataAspectRatio = [1,1,1];
ax.XLim = [290,310];  % 分析区在290-310米
ax.YLim = [0,2*3.5];
xlabel(ax,'x (m)'); ylabel(ax,'y (m)');

% 输入区：车流量
uicontrol(fig,'Style','text','String','车流量', ...
    'Units','normalized','Position',[0.05,0.18,0.15,0.05],'HorizontalAlignment','left','FontSize',10,'FontWeight','bold');
uicontrol(fig,'Style','text','String','(veh/h/lane)', ...
    'Units','normalized','Position',[0.05,0.15,0.15,0.05],'HorizontalAlignment','left','FontSize',10);
vehEdit = uicontrol(fig,'Style','edit','String','300', ...
    'Units','normalized','Position',[0.15,0.18,0.10,0.05],'Tag','vehRateEdit','BackgroundColor','white','FontWeight','bold');

% 输入区：行人流量
uicontrol(fig,'Style','text','String','行人流量', ...
    'Units','normalized','Position',[0.30,0.18,0.15,0.05],'HorizontalAlignment','left','FontSize',10,'FontWeight','bold');
uicontrol(fig,'Style','text','String','(ped/h)', ...
    'Units','normalized','Position',[0.30,0.15,0.15,0.05],'HorizontalAlignment','left','FontSize',10);
pedEdit = uicontrol(fig,'Style','edit','String','250', ...
    'Units','normalized','Position',[0.40,0.18,0.10,0.05],'Tag','pedRateEdit','BackgroundColor','white','FontWeight','bold');

% 放慢速度滑块
uicontrol(fig,'Style','text','String','减慢', ...
    'Units','normalized','Position',[0.55,0.18,0.15,0.05],'HorizontalAlignment','left','FontSize',10,'FontWeight','bold');
uicontrol(fig,'Style','text','String','(pause s)', ...
    'Units','normalized','Position',[0.55,0.15,0.15,0.05],'HorizontalAlignment','left','FontSize',10);
speedSlider = uicontrol(fig,'Style','slider', ...
    'Min',0,'Max',0.5,'Value',0,'SliderStep',[0.02 0.1], ...
    'Units','normalized','Position',[0.65,0.19,0.15,0.03],'Tag','pauseSlider','BackgroundColor','white');

% 清除数据按钮
uicontrol(fig,'Style','pushbutton','String','清除数据', ...
    'Units','normalized','Position',[0.82,0.18,0.13,0.05],'Callback',@(~,~)clearData(ax,fig),'FontSize',10,'FontWeight','bold');

% 按钮
uicontrol(fig,'Style','pushbutton','String','开始仿真', ...
    'Units','normalized','Position',[0.30,0.05,0.15,0.08], ...
    'Callback',@(~,~)startSim(ax,fig),'BackgroundColor',[0.5 0.6 1],'ForegroundColor','white','FontSize',10,'FontWeight','bold');

uicontrol(fig,'Style','pushbutton','String','结束仿真', ...
    'Units','normalized','Position',[0.55,0.05,0.15,0.08], ...
    'Callback',@(~,~)stopSim(fig),'BackgroundColor',[0.5 0.6 1],'ForegroundColor','white','FontSize',10,'FontWeight','bold');
end

function startSim(ax,fig)
% 读取输入流量、放慢时间
vehRate = str2double(findobj(fig,'Tag','vehRateEdit').String);
pedRate = str2double(findobj(fig,'Tag','pedRateEdit').String);
slider = findobj(fig,'Tag','pauseSlider');

% 参数
N_lanes=2; xc=300; D=3.5*N_lanes; laneW=3.5;
M=0; E_p=0; % 中央安全岛和电子警察
dt=0.1; T=7200; steps=ceil(T/dt);
analysis_area=[0,500];  % 仿真区域
visual_area=[290,310];  % 可视化区域

% 模型参数
kappa=0.41; lambda=0.2;
V1=6.75; V2=7.91; C1=0.13; C2=1.57; Lc=9; V_max=V1+V2;
v_ped_range=[0.5,1.5];

% 预分配
maxV=ceil(vehRate*N_lanes*T/3600);
maxP=ceil(pedRate*T/3600);
vehicles=nan(maxV,7); % [id, lane, x, v, yield, entry_time, exit_time]
peds=nan(maxP,6); % [id, position, wait_time, cross_time, status, v]
vehCnt=0; pedCnt=0;
vehArr=poissrnd((vehRate/3600)*dt,[N_lanes,steps]);
pedArr=poissrnd((pedRate/3600)*dt,[1,steps]);
vehG=gobjects(maxV,1);
pedG=gobjects(maxP,1);

% 清除旧输出
delete(findall(ax,'Type','text'));
delete(findall(ax,'Type','rectangle'));
delete(findall(ax,'Type','line'));

% 绘制路面与横道
for k=0:N_lanes, plot(ax,visual_area,[k,k]*laneW,'k'); end
yg=(N_lanes*laneW-D)/2;
% 绘制人行横道背景
rectangle(ax,'Position',[xc,yg,2,D],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none');
% 绘制人行横道白色条纹
stripe_width = 0.5; % 条纹宽度
stripe_spacing = 0.5; % 条纹间距
num_stripes = floor(D / (stripe_width + stripe_spacing));
for s = 0:num_stripes-1
    y_start = yg + s * (stripe_width + stripe_spacing);
    rectangle(ax,'Position',[xc,y_start,2,stripe_width],'FaceColor','white','EdgeColor','none');
end

% 主循环
for s=1:steps
    t=(s-1)*dt;
    pauseTime = slider.Value;

    % 生成车辆
    for lane=1:N_lanes
        if vehArr(lane,s)
            vehCnt=vehCnt+1;
            vehicles(vehCnt,:)=[vehCnt,lane,analysis_area(1),V1,0,t,nan];
            if vehicles(vehCnt,3)>=visual_area(1) && vehicles(vehCnt,3)<=visual_area(2)
                y0=(lane-0.5)*laneW;
                vehG(vehCnt)=rectangle(ax,'Position',[vehicles(vehCnt,3),y0-0.4,2,0.8],'FaceColor','b');
            end
        end
    end
    % 生成行人
    if pedArr(1,s)
        pedCnt=pedCnt+1;
        v_p = v_ped_range(1)+rand*(v_ped_range(2)-v_ped_range(1));
        peds(pedCnt,:)=[pedCnt,1,0,0,0,v_p];
        pedG(pedCnt)=plot(ax,xc-1,yg-0.5,'ro','MarkerFaceColor','r');
    end

    % 统计行人状态
    active_peds = ~isnan(peds(:,1));
    W_t = sum(peds(active_peds,5)==0);
    C_t = sum(peds(active_peds,5)==1);

    % 行人决策与更新
    for p=1:pedCnt
        if ~isnan(peds(p,1)) && peds(p,5)==0 % waiting
            accept=true(1,N_lanes);
            for lane=1:N_lanes
                G_l=Inf;
                for v=1:vehCnt
                    if ~isnan(vehicles(v,1)) && vehicles(v,2)==lane && vehicles(v,3)<xc
                        dist=xc-vehicles(v,3);
                        if vehicles(v,4)>0
                            temp_G=dist/vehicles(v,4);
                            G_l=min(G_l,temp_G);
                        end
                    end
                end
                U_il=6.365 + 2.678*G_l - 2.846*D + 0.058*peds(p,3) - 1.273*peds(p,2);
                p_il=1/(1+exp(-U_il));
                accept(lane)=(p_il>=0.5);
            end
            if all(accept)
                peds(p,5)=1; % crossing
            end
        end
        if ~isnan(peds(p,1))
            if peds(p,5)==1
                peds(p,4)=peds(p,4)+dt;
                ynew=yg + (peds(p,4)/(D/peds(p,6)))*D;
                set(pedG(p),'XData',xc,'YData',ynew);
                if peds(p,4)>=D/peds(p,6)
                    peds(p,5)=2; % finished
                    delete(pedG(p));
                end
            elseif peds(p,5)==0
                peds(p,3)=peds(p,3)+dt;
            end
        end
    end

    % 车辆让行判定与FVD跟驰
    for v=1:vehCnt
        if ~isnan(vehicles(v,1)) && isnan(vehicles(v,7))
            % 查找前车
            dx=Inf; dv=0; front_pass=true;
            for u=1:vehCnt
                if u==v || isnan(vehicles(u,1))
                    continue;
                end
                if vehicles(u,2)==vehicles(v,2) && vehicles(u,3)>vehicles(v,3)
                    d_temp=vehicles(u,3)-vehicles(v,3);
                    if d_temp<dx
                        dx=d_temp;
                        dv=vehicles(u,4)-vehicles(v,4);
                        front_pass=false;
                    end
                end
            end
            if dx<Inf && (vehicles(v,3)+dx)>=xc
                front_pass=true;
            end
            % 让行判定
            if vehicles(v,3)<xc && front_pass
                if C_t>0
                    vehicles(v,5)=1;
                elseif W_t>0
                    dist=xc-vehicles(v,3);
                    G_n=dist/max(vehicles(v,4),0.1);
                    Z_n=1.291 + 2.937*G_n - 8.462*N_lanes + 0.730*W_t + 3.008*M + 11.913*E_p;
                    vehicles(v,5)=(1/(1+exp(-Z_n))>=0.5);
                else
                    vehicles(v,5)=0;
                end
            else
                vehicles(v,5)=0;
            end
            % FVD模型
            if vehicles(v,3)>=xc
                Vopt=V1 + V2*tanh(C1*(dx-Lc)-C2);
                acc=kappa*(Vopt-vehicles(v,4));
            elseif vehicles(v,5)==1
                dn=xc-vehicles(v,3);
                Vpy=max(0,V1 + V2*tanh(C1*(dn-Lc)-C2));
                acc=kappa*(Vpy-vehicles(v,4)) - lambda*vehicles(v,4);
            else
                Vopt=V1 + V2*tanh(C1*(dx-Lc)-C2);
                acc=kappa*(Vopt-vehicles(v,4)) + lambda*dv;
            end
            % 速度位置更新
            vehicles(v,4)=max(0, min(V_max, vehicles(v,4) + acc*dt)); 
            vehicles(v,3) = vehicles(v,3) + vehicles(v,4)*dt + 0.5*acc*dt^2; 
            % 更新可视化
            if vehicles(v,3)>=visual_area(1) && vehicles(v,3)<=visual_area(2)
                if ~isgraphics(vehG(v))
                    y0=(vehicles(v,2)-0.5)*laneW;
                    vehG(v)=rectangle(ax,'Position',[vehicles(v,3),y0-0.4,2,0.8],'FaceColor','b');
                else
                    pos=get(vehG(v),'Position'); pos(1)=vehicles(v,3); set(vehG(v),'Position',pos);
                end
            elseif isgraphics(vehG(v))
                delete(vehG(v));
            end
            if vehicles(v,3)>=analysis_area(2)
                vehicles(v,7)=t;
                if isgraphics(vehG(v))
                    delete(vehG(v));
                end
            end
        end
    end
    drawnow;
    pause(pauseTime);
    if ~isvalid(fig), break; end
end
% 延误计算与输出
idxV=~isnan(vehicles(:,7)); 
free_time=(analysis_area(2)-analysis_area(1))/V_max;
vehDelays=vehicles(idxV,7)-vehicles(idxV,6)-free_time;
idxP=peds(:,5)==2; pedWaits=peds(idxP,3);
avgV=mean(vehDelays); avgP=mean(pedWaits);
str={sprintf('平均车辆延误: %.2f s',avgV),sprintf('平均行人延误: %.2f s',avgP)};
text(ax,0.5,0.9,str,'Units','normalized','FontSize',12,'Color','k');
end

function clearData(ax,~)
% 清除所有可视化对象和数据
delete(findall(ax,'Type','text'));
delete(findall(ax,'Type','rectangle'));
delete(findall(ax,'Type','line'));

% 重新绘制道路和人行横道
N_lanes=2; xc=300; D=3.5*N_lanes; laneW=3.5;
visual_area=[290,310];
for k=0:N_lanes, plot(ax,visual_area,[k,k]*laneW,'k'); end
yg=(N_lanes*laneW-D)/2;
rectangle(ax,'Position',[xc,yg,2,D],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none');
stripe_width = 0.5;
stripe_spacing = 0.5;
num_stripes = floor(D / (stripe_width + stripe_spacing));
for s = 0:num_stripes-1
    y_start = yg + s * (stripe_width + stripe_spacing);
    rectangle(ax,'Position',[xc,y_start,2,stripe_width],'FaceColor','white','EdgeColor','none');
end
drawnow;
end

function stopSim(fig)
close(fig);
end