clc; clear; close all;
%% 参数读取与设置
config;
filename = conf.filename;
[X, Y, Height] = SquareMap(filename);
[~, Xn] = size(X);
[~, Yn] = size(Y);

Height = Height';
start = [X(conf.start(1)), Y(conf.start(2)), Height(conf.start(1), conf.start(2)) + conf.start(3), conf.start(4), conf.start(5), conf.start(6)];
goal = [X(conf.goal(1)), Y(conf.goal(2)), Height(conf.goal(1), conf.goal(2)) + conf.goal(3), conf.goal(4), conf.goal(5), conf.goal(6)];
threshold = conf.threshold;
maxFailedAttempts = conf.maxFailedAttempts;
searchSize =  conf.search*[(goal(1) - start(1)), (goal(2) - start(2)), goal(3) - start(3), 0, 0, 0];
randnum=conf.randnum;
Tree_index=2;
failedAttempts = 0;
pathFound = false;
display = true;

RRTree = zeros(200,8);
RRTree(1,:)=[start, -1,0];

%% 绘制
figure(1)
meshz(X, Y, Height'); hold on
bar1 = scatter3(start(1), start(2), start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
bar2 = scatter3(goal(1), goal(2), goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
text(start(1), start(2), start(3), '  起点');
text(goal(1), goal(2), goal(3), '  终点');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('RRT算法UAV航迹规划路径');
% axis equal
% set(gcf,'unit','centimeters','position',[30 10 20 15]);
tic;
start(1)=start(1)-(searchSize(1)-(goal(1)-start(1)))/2;
start(2)=start(2)-(searchSize(2)-(goal(2)-start(2)))/2;

while failedAttempts <= maxFailedAttempts
    %% 选择随机点作为延展目标 50%几率随机 50%几率goal
    if rand < randnum
        sample = rand(1, 6) .* searchSize + start;
    else
        sample = goal;
    end

    %% 选择RRTree上距离sample最近的一点作为延展根节点
    [A, I] = min(distanceCost2(RRTree(:, 1:6), sample), [], 1);
    closestNode = RRTree(I(1), 1:6);
    %% 延展RRTree
    tmplot(1) = plot3([closestNode(1); sample(1)], [closestNode(2); sample(2)], [closestNode(3); sample(3)], 'LineWidth', 3);
    newPoint = extends(sample, closestNode, X, Y, Height,conf);
    % tmplot(2) = plot3([closestNode(1); newPoint(1)], [closestNode(2); newPoint(2)], [closestNode(3); newPoint(3)], 'LineWidth', 3);
    pause(0.05);
    delete(tmplot);
    %% 判断延展后的新点newPoint是否满足要求（碰撞检测）
    if ~checkPath(closestNode, newPoint, Height, X, Y)
        failedAttempts = failedAttempts + 1;
        RRTree(I(1),8)=RRTree(I(1),8)+1;
        RRTree(I(1),3)=RRTree(I(1),3)*1.1;
        continue;
    end 
    % 检测newPoint是否临近目标点
    if distanceCost(newPoint, goal) < 1.5*threshold, pathFound = true; break; end
    % 如果newPoint与之前RRTree上某一点的距离小于threshold说明newPoint的意义不大，舍弃
    [A, I2] = min(distanceCost2(RRTree(:, 1:6), newPoint), [], 1);
    if distanceCost2(newPoint, RRTree(I2(1), 1:6)) < 0.3 * threshold
        failedAttempts = failedAttempts + 1; continue; end
    %% 将newPoint加入RRTree
   % RRTree = [RRTree; newPoint I(1)]; % add node
    RRTree(Tree_index,:)=[newPoint, I(1),0];
    Tree_index=Tree_index+1;
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
    path = [RRTree(prev, 1:6); path];
    prev = RRTree(prev, 7);
end

bar3 = plot3(path(:, 1), path(:, 2), path(:, 3), 'LineWidth', 2, 'color', 'r');
% filPathX = [start(1), MovingAverage(path(2:end - 1, 1), 5), goal(1)];
% filPathY = [start(2), MovingAverage(path(2:end - 1, 2), 5), goal(2)];
% filPathZ = [start(3), MovingAverage(path(2:end - 1, 3), 5), goal(3)];
% bar4 = plot3(filPathX, filPathY, filPathZ, 'LineWidth', 3, 'color', 'g');
% legend([bar1, bar2, bar3, bar4], ["起始点", "终止点", "无人机航迹", "MA平滑后航迹"], 'Location', 'northwest');
% %% 存储轨迹
% writematrix([filPathX', filPathY', filPathZ'], [filename, 'rrt_path.csv']);
 %% 计算轨迹长度以及解算时间
 pathLength = 0;
 for i = 1:length(path(:, 1)) - 1, pathLength = pathLength + distanceCost(path(i, 1:3), path(i + 1, 1:3)); end % calculate path length
 fprintf('运行时间：%d \n路径长度=%d\n GS:%f°\n LS:%f°', toc, pathLength, calGs(path) / pi * 180, calLs(path) / pi * 180);

t1=saves('output','path',0);
t2=saves('output','RRTree',1);

saveas(1,t1);
save(t2,'RRTree');
%% 碰撞检测函数

function flag = checkPath(start, endp, Height, X, Y)
    flag = true;
    start_insdex = [find_closest(start(1), X), find_closest(start(2), Y)];
    end_insdex = [find_closest(endp(1), X), find_closest(endp(2), Y)];
    [~, Xn] = size(X);
    [~, Yn] = size(Y);

    if end_insdex(1, 1) ~= start_insdex (1, 1)
        k_index = (end_insdex(1, 2) - start_insdex(1, 2)) / (end_insdex(1, 1) - start_insdex(1, 1));
    else
        k_index = 999999999;
    end

    if abs(k_index) > 1
        x_max = Yn;
        y_max = Xn;
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
        x_max = Xn;
        y_max = Yn;
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

        if y_up > y_max
            y_up = y_max;
        end

        if new_Height(start_insdex(1, 1) + i, y_up) > h
            flag = false; break;
        end

        if new_Height(start_insdex(1, 1) + i, y_down) > h
            flag = false; break;
        end

    end

end

function newPoint = extends(sample, closestNode, X, Y, Height,conf)
    % movingVec = [sample(1) - closestNode(1), sample(2) - closestNode(2), ]; %sample(3) - closestNode(3)];
    % movingVec = movingVec / sqrt(sum(movingVec .^ 2)); %单位化
    % newPoint = closestNode(1:2) + stepSize * movingVec;
    % newPoint(3) = Height(find_closest(newPoint(1), X), find_closest(newPoint(2), Y)) + height_limit;

    height_limit = conf.height_limit;
    deltaT = conf.deltaT;
    g = conf.g;
    v = conf.v;
    GammaMax = conf.GammaMax;
    GammaMin = conf.GammaMin;
    pitchMax = conf.pitchMax;
    pitchMin = conf.pitchMin;

    GammaStep = conf.GammaStep; %滚转角最大步长
    pitchstep = conf.pitchstep; %俯仰角最大步长
    movingVec = [sample(1) - closestNode(1), sample(2) - closestNode(2), ]; %sample(3) - closestNode(3)];
    phi1 = atan(movingVec(2) / movingVec(1));

    if movingVec(2) > 0 && movingVec(1) < 0
        phi1 = phi1 + 3.1416;
    elseif movingVec(2) < 0 && movingVec(1) < 0
        phi1 = phi1 - 3.1416;
    end

    phi = closestNode(4);
    deltaPhi = (-phi + phi1);

    while deltaPhi > 3.1416
        deltaPhi = deltaPhi - 6.2832;

    end

    while deltaPhi <- 3.1416
        deltaPhi = deltaPhi + 6.2832;

    end

    newGamma = atan(deltaPhi * v / g / deltaT);

    if (newGamma - closestNode(5)) > GammaStep
        newGamma = closestNode(5) + GammaStep;

    elseif (newGamma - closestNode(5)) <- GammaStep
        newGamma = closestNode(5) - GammaStep;

    end

    if newGamma > GammaMax
        newGamma = GammaMax;
    elseif newGamma < GammaMin
        newGamma = GammaMin;
    end

    rotation = [cos(phi) -sin(phi);
                sin(phi) cos(phi); ];

    if newGamma ~= 0
        Rou = tan(newGamma) * g / (v ^ 2); %计算水平转弯曲率
        deltaPhi = Rou * v * deltaT;
        newPhi = deltaPhi + phi;
        a = [sin(deltaPhi) / Rou; ((1 - cos(deltaPhi)) / Rou)];
    else
        newPhi = phi;
        a = [v * deltaT; 0];
    end

    temp = (rotation * a)' + [closestNode(1), closestNode(2)];

    targetHeight = Height(find_closest(temp(1), X), find_closest(temp(2), Y)) + height_limit;
    distanceee = distanceCost([temp(1), temp(2)], [closestNode(1), closestNode(2)]);
    pitchangle = atan((targetHeight - closestNode(3)) / distanceee);

    if pitchangle > pitchMax
        pitchangle = pitchMax;
    elseif pitchangle < pitchMin
        pitchangle = pitchMin;
    end

    if pitchangle - closestNode(6) > pitchstep
        pitchangle = closestNode(5) + pitchstep;

        % z = tan(pitchangle) * distanceee + closestNode(3);

    elseif pitchangle - closestNode(6) <- pitchstep
        pitchangle = closestNode(5) - pitchstep;

        % z = tan(pitchangle) * distanceee + closestNode(3);
    else
        % z = tan(pitchangle) * distanceee + closestNode(3);

        % z = targetHeight;
    end

    z = pitchangle * distanceee + closestNode(3);

    newPoint = [temp(1), temp(2), z, newPhi, newGamma, pitchangle];
end


function h = distanceCost(a, b)
    h = sqrt(sum((a - b) .^ 2, 2));
end

function h = distanceCost2(a, b)
    a1 = a;
    b1 = b;
    a1(:, 4:6) = 0 * a(:, 4:6);
    b1(:, 4:6) = 0 * b(:, 4:6);
    h = sqrt(sum((a1 - b1) .^ 2, 2));

end
