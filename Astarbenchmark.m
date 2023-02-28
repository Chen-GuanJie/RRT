% 毕业设计对比算法A* algorithm实现
clc; clear; close all;
%% 参数读取与设置
filename = 'data/Output_500';
[X, Y, Height] = SquareMap(filename);
% X = normalize(X);
% Y = normalize(Y);
% Height = normalize(Height);
[~, Xn] = size(X);
[~, Yn] = size(Y);

Height = Height';
start = [X(280), Y(80), Height(280, 80) + 500];
goal = [X(70), Y(600), Height(70, 600) + 600];

Alldirec = [[1, 0, 0]; [0, 1, 0]; [0, 0, 1]; [-1, 0, 0]; [0, -1, 0]; [0, 0, -1]; ...
                                                                          [1, 1, 0]; [1, 0, 1]; [0, 1, 1]; [-1, -1, 0]; [-1, 0, -1]; [0, -1, -1]; ...
                                                                          [1, -1, 0]; [-1, 1, 0]; [1, 0, -1]; [-1, 0, 1]; [0, 1, -1]; [0, -1, 1]; ...
                                                                          [1, 1, 1]; [-1, -1, -1]; [1, -1, -1]; [-1, 1, -1]; [-1, -1, 1]; [1, 1, -1]; ...
                                                                          [1, -1, 1]; [-1, 1, 1]];
threshold = 5000;
stop = threshold * 1.5;
g = [start, 0; goal, inf]; % 每一行前三个数为点坐标，第四个数为路径耗散
Path = [];
Parent = [];
Open = [start, g(findIndex(g, start), 4) + getDist(start, goal)];
%% 绘制障碍环境
figure(1)
meshz(X, Y, Height'); hold on
bar1 = scatter3(start(1), start(2), start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
bar2 = scatter3(goal(1), goal(2), goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
text(start(1), start(2), start(3), '  起点');
text(goal(1), goal(2), goal(3), '  终点');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('A*算法UAV航迹规划路径');
axis equal
% set(gcf,'unit','centimeters','position',[30 10 20 15]);
%% 主循环
tic;

while ~isempty(Open)
    [xi, index] = findMin(Open);
    Open(index, :) = [];

    if getDist(xi, goal) < stop
        break;
    end

    children = getChildren(xi, Alldirec, threshold, Height, X, Y);
    %scatter3(children(:, 1), children(:, 2), children(:, 3), 1, 'filled', 'o');
    drawnow;
    [n, ~] = size(children);

    for i = 1:n
        child = children(i, :);

        if findIndex(g, child) == 0 % child不在g
            g = [g; child, inf];
        end

        a = g(findIndex(g, xi), 4) + getDist(xi, child);

        if a < g(findIndex(g, child), 4)
            g(findIndex(g, child), 4) = a;
            Parent = setParent(Parent, child, xi);
            Open = setOpen(Open, child, a, goal);
        end

    end

end

lastPoint = xi;
%% 回溯轨迹
x = lastPoint;
Path = x;
[n, ~] = size(Parent);

while any(x ~= start)

    for i = 1:n

        if Parent(i, 1:3) == x
            Path = [Parent(i, 4:6); Path];
            break;
        end

    end

    x = Parent(i, 4:6);
end

bar3 = plot3([Path(:, 1); goal(1)], [Path(:, 2); goal(2)], [Path(:, 3); goal(3)], 'LineWidth', 3, 'color', 'r');
filPathX = [start(1), MovingAverage(Path(2:end, 1), 5), goal(1)];
filPathY = [start(2), MovingAverage(Path(2:end, 2), 5), goal(2)];
filPathZ = [start(3), MovingAverage(Path(2:end, 3), 5), goal(3)];
bar4 = plot3(filPathX, filPathY, filPathZ, 'LineWidth', 3, 'color', 'g');
legend([bar1, bar2, bar3, bar4], ["起始点", "终止点", "无人机航迹", "MA平滑后航迹"], 'Location', 'northwest');
%% 完整path
path = [Path; goal];
%% 计算轨迹距离
pathLength = 0;
[n, ~] = size(Path);

for i = 1:n - 1
    pathLength = pathLength + getDist(Path(i, :), Path(i + 1, :));
end

pathLength = pathLength + getDist(Path(end, :), goal);
fprintf('计算时间:%f秒\n路径的长度为:%f\n GS:%f°\n LS:%f°', toc, pathLength, calGs(path) / pi * 180, calLs(path) / pi * 180);
%% 存储轨迹
csvwrite('F:\MasterDegree\毕业设计\实验数据\静态环境轨迹联合绘制/Astar_path.csv', [filPathX', filPathY', filPathZ']);
%% 函数
function children = getChildren(pos, Alldirec, step, Height, X, Y)
    allchild = [];
    [n, ~] = size(Alldirec);

    for i = 1:n
        direc = Alldirec(i, :);
        child = pos + direc * step;

        if ~checkPath(child, Height, X, Y)
            continue;
        end

        allchild = [allchild; child];
    end

    children = allchild;
end

function flag = checkPath(child, Height, X, Y)
    flag = true;
    child_insdex = [find_closest(child(1), X), find_closest(child(2), Y)];

    if Height(child_insdex(1), child_insdex(2)) > child(3)
        flag = false;
    end

end

function Par = setParent(Parent, xj, xi)
    [n, ~] = size(Parent);

    if n == 0
        Par = [xj, xi];
    else

        for i = 1:n

            if Parent(i, 1:3) == xj
                Parent(i, 4:6) = xi;
                Par = Parent;
                break;
            end

            if i == n
                Par = [Parent; xj, xi];
            end

        end

    end

end

function Ope = setOpen(Open, child, a, goal)
    [n, ~] = size(Open);

    if n == 0
        Ope = [child, a + getDist(child, goal)];
    else

        for i = 1:n

            if Open(i, 1:3) == child
                Open(i, 4) = a + getDist(child, goal);
                Ope = Open;
            end

            if i == n
                Ope = [Open; child, a + getDist(child, goal)];
            end

        end

    end

end

function h = heuristic(pos, goal)
    h = max([abs(goal(1) - pos(1)), abs(goal(2) - pos(2)), abs(goal(3) - pos(3))]);
end

function index = findIndex(g, pos)
    [n, ~] = size(g);
    index = 0; % 表示没有找到索引

    for i = 1:n

        if g(i, 1:3) == pos
            index = i; % 索引为i
            break;
        end

    end

end

function d = getDist(x, y)
    d = sqrt(sum((x - y) .^ 2));
end

function [pos, index] = findMin(Open)
    [~, index] = min(Open(:, 4));
    pos = Open(index, 1:3);
end

function index = find_closest(x, list)
    a = abs(list - x);
    [~, index] = min(a);
end

function LS = calLs(path)
    % 此函数计算整个过程中的最大飞行方向变化
    [n, ~] = size(path);
    mu1 = 0.5; mu2 = 0.5; % 对偏航角和爬升角的加权因子
    Max = 0;

    for i = 2:n - 1
        qBefore = path(i - 1, :);
        q = path(i, :);
        qNext = path(i + 1, :);
        % 计算qBefore到q航迹角x1,gam1
        qBefore2q = q - qBefore;
        gam1 = asin(qBefore2q(3) / sqrt(sum(qBefore2q .^ 2)));

        if qBefore2q(1) ~= 0 || qBefore2q(2) ~= 0
            x1 = asin(abs(qBefore2q(2) / sqrt(qBefore2q(1) ^ 2 + qBefore2q(2) ^ 2)));
        else
            x1 = 0;
        end

        % 计算q到qNext航迹角x2, gam2
        q2qNext = qNext - q;
        gam2 = asin(q2qNext(3) / sqrt(sum(q2qNext .^ 2)));

        if q2qNext(1) ~= 0 || q2qNext(2) ~= 0
            x2 = asin(abs(q2qNext(2) / sqrt(q2qNext(1) ^ 2 + q2qNext(2) ^ 2)));
        else
            x2 = 0;
        end

        % 根据不同象限计算矢量相对于x正半轴的角度 0-2 * pi
        if qBefore2q(1) > 0 && qBefore2q(2) > 0
            x1 = x1;
        end

        if qBefore2q(1) < 0 && qBefore2q(2) > 0
            x1 = pi - x1;
        end

        if qBefore2q(1) < 0 && qBefore2q(2) < 0
            x1 = pi + x1;
        end

        if qBefore2q(1) > 0 && qBefore2q(2) < 0
            x1 = 2 * pi - x1;
        end

        if qBefore2q(1) > 0 && qBefore2q(2) == 0
            x1 = 0;
        end

        if qBefore2q(1) == 0 && qBefore2q(2) > 0
            x1 = pi / 2;
        end

        if qBefore2q(1) < 0 && qBefore2q(2) == 0
            x1 = pi;
        end

        if qBefore2q(1) == 0 && qBefore2q(2) < 0
            x1 = 3 * pi / 2;
        end

        if q2qNext(1) > 0 && q2qNext(2) > 0
            x2 = x2;
        end

        if q2qNext(1) < 0 && q2qNext(2) > 0
            x2 = pi - x2;
        end

        if q2qNext(1) < 0 && q2qNext(2) < 0
            x2 = pi + x2;
        end

        if q2qNext(1) > 0 && q2qNext(2) < 0
            x2 = 2 * pi - x2;
        end

        if q2qNext(1) > 0 && q2qNext(2) == 0
            x2 = 0;
        end

        if q2qNext(1) == 0 && q2qNext(2) > 0
            x2 = pi / 2;
        end

        if q2qNext(1) < 0 && q2qNext(2) == 0
            x2 = pi;
        end

        if q2qNext(1) == 0 && q2qNext(2) < 0
            x2 = 3 * pi / 2;
        end

        % 判断角度变化是否超过Max
        if abs(x1 - x2) >= pi
            deltaX = 2 * pi - abs(x1 - x2);
        else
            deltaX = abs(x1 - x2);
        end

        if mu1 * abs(deltaX) + mu2 * abs(gam2 - gam1) > Max
            Max = mu1 * abs(deltaX) + mu2 * abs(gam2 - gam1);
        end

    end

    LS = Max;
end

function GS = calGs(path)
    % 这个函数用来计算一条轨迹的GS（平均转角弯度）
    [n, ~] = size(path);
    mu1 = 0.5; mu2 = 0.5; % 对偏航角和爬升角的加权因子
    GsSum = 0;

    for i = 2:n - 1
        qBefore = path(i - 1, :);
        q = path(i, :);
        qNext = path(i + 1, :);
        % 计算qBefore到q航迹角x1,gam1
        qBefore2q = q - qBefore;
        gam1 = asin(qBefore2q(3) / sqrt(sum(qBefore2q .^ 2)));

        if qBefore2q(1) ~= 0 || qBefore2q(2) ~= 0
            x1 = asin(abs(qBefore2q(2) / sqrt(qBefore2q(1) ^ 2 + qBefore2q(2) ^ 2)));
        else
            x1 = 0;
        end

        % 计算q到qNext航迹角x2, gam2
        q2qNext = qNext - q;
        gam2 = asin(q2qNext(3) / sqrt(sum(q2qNext .^ 2)));

        if q2qNext(1) ~= 0 || q2qNext(2) ~= 0
            x2 = asin(abs(q2qNext(2) / sqrt(q2qNext(1) ^ 2 + q2qNext(2) ^ 2)));
        else
            x2 = 0;
        end

        % 根据不同象限计算矢量相对于x正半轴的角度 0-2 * pi
        if qBefore2q(1) > 0 && qBefore2q(2) > 0
            x1 = x1;
        end

        if qBefore2q(1) < 0 && qBefore2q(2) > 0
            x1 = pi - x1;
        end

        if qBefore2q(1) < 0 && qBefore2q(2) < 0
            x1 = pi + x1;
        end

        if qBefore2q(1) > 0 && qBefore2q(2) < 0
            x1 = 2 * pi - x1;
        end

        if qBefore2q(1) > 0 && qBefore2q(2) == 0
            x1 = 0;
        end

        if qBefore2q(1) == 0 && qBefore2q(2) > 0
            x1 = pi / 2;
        end

        if qBefore2q(1) < 0 && qBefore2q(2) == 0
            x1 = pi;
        end

        if qBefore2q(1) == 0 && qBefore2q(2) < 0
            x1 = 3 * pi / 2;
        end

        if q2qNext(1) > 0 && q2qNext(2) > 0
            x2 = x2;
        end

        if q2qNext(1) < 0 && q2qNext(2) > 0
            x2 = pi - x2;
        end

        if q2qNext(1) < 0 && q2qNext(2) < 0
            x2 = pi + x2;
        end

        if q2qNext(1) > 0 && q2qNext(2) < 0
            x2 = 2 * pi - x2;
        end

        if q2qNext(1) > 0 && q2qNext(2) == 0
            x2 = 0;
        end

        if q2qNext(1) == 0 && q2qNext(2) > 0
            x2 = pi / 2;
        end

        if q2qNext(1) < 0 && q2qNext(2) == 0
            x2 = pi;
        end

        if q2qNext(1) == 0 && q2qNext(2) < 0
            x2 = 3 * pi / 2;
        end

        % 累计GS
        if abs(x1 - x2) >= pi
            deltaX = 2 * pi - abs(x1 - x2);
        else
            deltaX = abs(x1 - x2);
        end

        GsSum = GsSum + (mu1 * abs(deltaX) + mu2 * abs(gam1 - gam2));
    end

    % 计算GS
    GS = GsSum / (n - 2);
end
function [x_data, y_data, z_data] = SquareMap(filename)
    load(filename, 'dem_data')
    dem_data = sortrows(dem_data);
    x = dem_data(:, 1);
    y = dem_data(:, 2);
    z = dem_data(:, 3);
    [X, Xn] = grid(x);
    [Y, Yn] = grid(y);
    x_index = get_uniqu_index(x);
    [x_size, ~] = size(x);
    x_index(Xn) = x_size;
    x_num = zeros(Xn, 1);
    x_num(1) = x_index(1);
    x_num(2:Xn) = diff(x_index);
    Height = zeros(Xn, Yn);

    for i = 1:1:Xn

        for j = 1:x_num(i)

            if i == 1
                start = 1;
            else
                start = x_index(i - 1) + 1;
            end

            stop = x_index(i);

            if (stop - start + 1) == Yn
                Height(i, :) = z(start:stop);
            else
                a = find(Y == y(start));
                b = find(Y == y(stop));

                if (b - a) == stop - start
                    Height(i, a:b) = z((start:stop));
                else
                    m = 1;

                    for k = 1:stop - start + 1

                        while Y(m) ~= y(k)
                            m = m + 1;
                        end

                        Height(i, m) = z(start + k - 1);
                    end

                end

            end

        end

    end

    x_data = X;
    y_data = Y;
    z_data = Height';
end

function x_index = get_uniqu_index(x)
    x_diff = diff(x);
    x_index = find(x_diff ~= 0);
end

function [g, num] = grid(x)
    x_unique = unique(x);
    a = size(x_unique);
    num = a(1, 1);
    g = sortrows(x_unique)';
end
function res = MovingAverage(input,N)
    %% input为平滑前序列(列向量和行向量均可)；N为平滑点数（奇数）；res返回平滑后的序列(默认行向量)。
    sz = max(size(input));
    n = (N-1)/2;
    res = [];
    for i = 1:length(input)
        if i <= n
            res(i) = sum(input(1:2*i-1))/(2*i-1);
        elseif i < length(input)-n+1
            res(i) = sum(input(i-n:i+n))/(2*n+1);
        else
            temp = length(input)-i+1;
            res(i) = sum(input(end-(2*temp-1)+1:end))/(2*temp-1);
        end
    end
    end
    
    