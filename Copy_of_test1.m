clc; clear; close all;
%% 参数读取与设置
load('dem_data.mat')
dem_data = sortrows(dem_data);
x = dem_data(:, 1);
y = dem_data(:, 2);
z = dem_data(:, 3);
[X, Xn] = grid(x);
[Y, Yn] = grid(y);
X = X - X(1);
Y = (Y - Y(1)) * 10;
Yn = 280;
Height = zeros(Xn, Yn);

for i = 0:1:Xn - 1
    t = dem_data((i * Yn + 1):(i + 1) * Yn, 3)';
    Height(i + 1, :) = t / 1000;
end

pai = 3.1415926;
max_phi = +pai;
min_phi = -pai;
max_gamma = 20/180 * pai;
min_gamma = -20/180 * pai;
max_pitch = 40/180 * pai;
min_pitch = -40/180 * pai;
% scatter3(x,y,z,".")
Y = Y(1, 1:280);

start = [X(30), Y(30), Height(30, 30) + 0.2, 0, 0, 0];
goal = [X(Xn - 30), Y(Yn - 30), Height(Xn - 30, Yn - 30) + 0.1, 0, 0, 0];

stepSize = 0.02;
threshold = 0.01;
maxFailedAttempts = 10000;
searchSize = 1.3 * [goal(1) - start(1), goal(2) - start(2), goal(3) - start(3)];
searchAngle = [max_phi - min_phi, max_gamma - min_gamma, max_pitch - min_pitch];
RRTree = double([start, -1]);
failedAttempts = 0;
pathFound = false;
display = true;
%% 绘制
figure(1)
meshz(X, Y, Height'); hold on
bar1 = scatter3(start(1), start(2), start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
bar2 = scatter3(goal(1), goal(2), goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
text(start(1), start(2), start(3), '  起点');
text(goal(1), goal(2), goal(3), '  终点');
xlabel('x(km)'); ylabel('y(km)'); zlabel('z(km)');
title('RRT算法UAV航迹规划路径');
% axis equal
% set(gcf,'unit','centimeters','position',[30 10 20 15]);
%% 主循环
tic;

while failedAttempts <= maxFailedAttempts
    %% 选择随机点作为延展目标 50%几率随机 50%几率goal
    if rand < 0.5
        sample(1, 1:3) = rand(1, 3) .* searchSize + start(1, 1:3);
        sample(1, 4:6) = rand(1, 3) .* searchAngle + [min_phi, min_gamma, min_pitch];
    else
        sample = goal;
    end

    %% 选择RRTree上距离sample最近的一点作为延展根节点
    [A, I] = min(distanceCost(RRTree(:, 1:6), sample), [], 1);
    closestNode = RRTree(I(1), 1:6);
    %% 延展RRTree
    %     movingVec = [sample(1)-closestNode(1),sample(2)-closestNode(2),sample(3)-closestNode(3)];
    %     movingVec = movingVec/sqrt(sum(movingVec.^2));  %单位化
    %     newPoint = closestNode + stepSize * movingVec;
    newPoint = getNextPoint2(closestNode, sample);
    %     plot3(newPoint(1),newPoint(2),newPoint(3),'.');
    %% 判断延展后的新点newPoint是否满足要求（碰撞检测）
    if ~checkPath(closestNode, newPoint, Height, X, Y)
        failedAttempts = failedAttempts + 1;
        continue;
    end

    % 检测newPoint是否临近目标点
    if distanceCost(newPoint, goal) < threshold, pathFound = true; break; end
    % 如果newPoint与之前RRTree上某一点的距离小于threshold说明newPoint的意义不大，舍弃
    [A, I2] = min(distanceCost(RRTree(:, 1:6), newPoint), [], 1);
    if distanceCost(newPoint, RRTree(I2(1), 1:6)) < threshold, failedAttempts = failedAttempts + 1; continue; end
    %% 将newPoint加入RRTree
    RRTree = [RRTree; newPoint I(1)]; % add node
    failedAttempts = 0;
    if display, plot3([closestNode(1); newPoint(1)], [closestNode(2); newPoint(2)], [closestNode(3); newPoint(3)], 'LineWidth', 2); end
    pause(0.05);
end

if display && pathFound, plot3([closestNode(1); goal(1)], [closestNode(2); goal(2)], [closestNode(3); goal(3)], 'LineWidth', 2); end
if ~pathFound, error('no path found. maximum attempts reached'); end
%% 回溯轨迹
path = goal;
prev = I(1);

while prev > 0
    path = [RRTree(prev, 1:3); path];
    prev = RRTree(prev, 4);
end

bar3 = plot3(path(:, 1), path(:, 2), path(:, 3), 'LineWidth', 3, 'color', 'r');
filPathX = [start(1), MovingAverage(path(2:end - 1, 1), 5), goal(1)];
filPathY = [start(2), MovingAverage(path(2:end - 1, 2), 5), goal(2)];
filPathZ = [start(3), MovingAverage(path(2:end - 1, 3), 5), goal(3)];
bar4 = plot3(filPathX, filPathY, filPathZ, 'LineWidth', 3, 'color', 'g');
legend([bar1, bar2, bar3, bar4], ["起始点", "终止点", "无人机航迹", "MA平滑后航迹"], 'Location', 'northwest');
%% 存储轨迹
% csvwrite('F:\MasterDegree\毕业设计\实验数据\静态环境轨迹联合绘制/rrt_path.csv',[filPathX',filPathY', filPathZ']);
%% 计算轨迹长度以及解算时间
pathLength = 0;
for i = 1:length(path(:, 1)) - 1, pathLength = pathLength + distanceCost(path(i, 1:3), path(i + 1, 1:3)); end % calculate path length
fprintf('运行时间：%d \n路径长度=%d\n GS:%f°\n LS:%f°', toc, pathLength, calGs(path) / pi * 180, calLs(path) / pi * 180);
%% 碰撞检测函数
function flag = checkPath(start, endp, Height, X, Y)
    flag = true;
    start_insdex = [find_closest(start(1), X), find_closest(start(2), Y)];
    end_insdex = [find_closest(endp(1), X), find_closest(endp(2), Y)];

    if end_insdex(1, 1) ~= start_insdex (1, 1)
        k_index = (end_insdex(1, 2) - start_insdex(1, 2)) / (end_insdex(1, 1) - start_insdex(1, 1));
    else
        k_index = 999999999;
    end

    if abs(k_index) > 1
        new_Height = Height';
        xx = start_insdex(1, 1);
        yy = start_insdex(1, 2);
        start_insdex(1, 1) = yy;
        start_insdex(1, 2) = xx;
        xx = end_insdex(1, 1);
        yy = end_insdex(1, 2);
        end_insdex(1, 1) = yy;
        end_insdex(1, 2) = xx;
        k_index = 1 / k_index;
    else
        new_Height = Height;
    end

    if end_insdex(1, 1) - start_insdex(1, 1) < 0
        increase = -1;
    elseif end_insdex(1, 1) - start_insdex(1, 1) > 0
        increase = 1;
    else
        return
    end

    for i = 0:increase:(end_insdex(1, 1) - start_insdex(1, 1))
        y_up = ceil((i) * k_index + start_insdex(1, 2));
        y_down = floor((i) * k_index + start_insdex(1, 2));
        h = start(1, 3) + (endp(1, 3) - start(1, 3)) * i / (end_insdex(1, 1) - start_insdex(1, 1));

        if new_Height(start_insdex(1, 1) + i, y_up) > h
            flag = false; break;
        end

        if new_Height(start_insdex(1, 1) + i, y_down) > h
            flag = false; break;
        end

    end

end

%     for x=start(1,1):0.001:endp(1,1)
%         y=(x-start(1,1))/(endp(1,1)-start(1,1))*(endp(1,2)-start(1,2));
%         z=(x-start(1,1))/(endp(1,1)-start(1,1))*(endp(1,3)-start(1,3));
%
%         temp=abs(map(:,1)-x);
%         map_x=min(temp);
%         [map_x_index,~]=find(temp==map_x);
%
%         temp=abs(map(:,2)-y);
%         map_y=min(temp);
%         [map_y_index,~]=find(temp==map_y);
%
%         map_x_size=size(map,1);
%
%         if z<map((map_y_index-1)*map_x_size+map_x_index,3)
%             flag=false;
%         end
%     end

function new = cut_map(start, endp, map)
    %截取地图
    x_min = min([start(1, 1), endp(1, 1)]);
    x_max = max([start(1, 1), endp(1, 1)]);
    y_min = min([start(1, 2), endp(1, 2)]);
    y_max = max([start(1, 2), endp(1, 2)]);

    x = map(map(:, 1) > x_min & map(:, 1) < x_max);
    y = map(map(:, 2) > y_min & map(:, 2) < y_max);
    new = [x y];
end

function h = distanceCost(a, b)
    h = sqrt(sum((a - b) .^ 2, 2));
end

function [g, num] = grid(x)
    x_unique = unique(x);
    a = size(x_unique);
    num = a(1, 1);
    g = sortrows(x_unique)';
end

function index = find_closest(x, list)
    a = abs(list - x);
    mini = min(a);
    index = find(a == mini);
end
