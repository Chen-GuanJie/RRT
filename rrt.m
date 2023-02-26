classdef rrt < handle

    properties (SetAccess = private)
        tree      %节点位置    % cost parentid id v
        parent    %父节点索引
        time_cost %时间消耗
        energe_cost %能量消耗
        velocity  %速度

        maps
        robot
        start
        goal
        threshold
        maxFailedAttempts
        searchSize
        searchbBase
        randnum
        edges
        tmpclose

        nearNodes
        distances
        newNode
        failedAttempts
        nodenum

        compare_table
    end

    methods (Access = private)

        function sample = get_sample(this)
            %采样
            if rand < this.randnum
                sample = rand(1, 6) .* this.searchSize + this.searchbBase;
            else
                sample = this.goal;
            end

        end

        function flag = neighbors(this, dis)
            %找附近的点
            flag = 0;
            this.compare_table = this.tree(1:this.nodenum - 1, 1:6) - this.newNode;
            this.distances = sum(this.compare_table(:, 1:3) .^ 2, 2);
            this.nearNodes = this.tree(find(this.distances < dis), :);
            [~, index] = min(this.distances);
            tmpdis = sqrt(this.distances(index(1)));

            if tmpdis < this.threshold %太近
                % this.failedAttempts = this.failedAttempts + 1;
                flag = 1;
                this.tmpclose(end + 1) = tmpdis;

            end

        end

        function [node, index] = get_closest(this, sample)
            %找最近点
            this.compare_table = this.tree(1:this.nodenum - 1, 1:6) - sample;
            [~, I] = min(sum(this.compare_table(:, 1:3) .^ 2, 2));
            index = I(1);
            node = this.tree(index, :);
        end

        function extends(this, sample, closestNode)
            %延申
            this.newNode = this.robot.transfer(sample, closestNode, this.maps);
        end

        function cost = calc_cost(~, from_node, dest_node)
            %计算相邻两点的代价
            cost_dist = norm(from_node(1:2) - dest_node(1:2));
            cost_hight = 3 * norm(from_node(3) - dest_node(3));
            cost_angle = norm(from_node(4:6) - dest_node(4:6));
            %consumption = this.robot.calc_consumption();
            cost = norm([cost_dist, cost_angle, cost_hight]);
        end

        function flag = check_newNode(this)
            flag = 0;

            % if ~this.maps.checkPath(closestNode, this.newNode)
            %     this.failedAttempts = this.failedAttempts + 1;
            %     % RRTree(I(1), 8) = RRTree(I(1), 8) + 1;
            %     % RRTree(I(1), 3) = RRTree(I(1), 3) * 1.1;
            %     flag = 1;

            % else
            if norm(this.newNode(1:3) - this.goal(1:3)) < 2 * this.threshold
                flag = 2;
            end

            % 如果newPoint与之前RRTree上某一点的距离小于threshold说明newPoint的意义不大，舍弃
            % [~, index] = min(this.distances);

            %if norm(this.tree(index(1), 1:3) - this.newNode(1:3)) < 1 * this.threshold
            % if sqrt(this.distances(index(1))) < this.threshold
            %     this.failedAttempts = this.failedAttempts + 1;
            %     flag = 1;

            % end

        end

        function insert_node(this, parent_id)
            %插入节点
            this.newNode(8) = parent_id;
            this.newNode(9) = this.nodenum;
            this.tree(this.nodenum, 1:9) = this.newNode;
            this.nodenum = this.nodenum + 1;
        end

        function trace_back(this, id)
            %回溯轨迹
            path = this.goal;
            prev = id;

            while prev > 0
                path = [this.tree(prev, 1:6); path];
                prev = this.tree(prev, 8);
            end

            bar3 = plot3(path(:, 1), path(:, 2), path(:, 3), 'LineWidth', 2, 'color', 'g');

        end

        function new_parent_id = choose_parent(this, closestNode)
            %RRT star 找新的父节点
            if this.maps.checkPath(closestNode, this.newNode)
                mini_cost = this.calc_cost(closestNode, this.newNode) + closestNode(7);
                new_parent_id = closestNode(9);
            else
                mini_cost = 0;
                new_parent_id = 0;
            end

            i = 1;
            n = numel(this.nearNodes(:, 1));

            while i <= n
            

                if (this.maps.checkPath(this.nearNodes(i, :), this.newNode)) %无碰撞

                    if (this.robot.transferable(this.nearNodes(i, :), this.newNode)) %可转移

                        tmpcost = this.calc_cost(this.nearNodes(i, :), this.newNode) + this.nearNodes(i, 7);

                        if tmpcost < mini_cost
                            mini_cost = tmpcost;
                            new_parent_id = this.nearNodes(i, 9);
                        end

                    end

                    i = i + 1;
                else %删去集合中有碰撞的附近点
                    this.nearNodes(i, :) = [];
                    i = i - 1;
                    n = n - 1;
                    if i==0
break;
                    end
                end

            end

            this.newNode(7) = mini_cost;

        end

        function replot = rewire(this, new_parent_id)
            %重布线
            replot = [];

            for i = 1:numel(this.nearNodes(:, 1))

                if new_parent_id ~= this.nearNodes(i, 8) %不是新节点的父节点

                    if this.robot.transferable(this.newNode, this.nearNodes(i, :))
                        tmpcost = this.calc_cost(this.newNode, this.nearNodes(i, :)) + this.newNode(7);

                        if tmpcost < this.nearNodes(i, 7)
                            % id parentid newparentid
                            replot (end + 1, :) = [this.nearNodes(i, 9) this.nearNodes(i, 8) this.newNode(9)];
                            this.nearNodes(i, 7) = tmpcost;
                            this.nearNodes(i, 8) = this.newNode(9);
                        end

                    end

                end

            end

        end

        function redisplay(this, replot)

            if ~isempty(replot)

                for i = 1:numel(replot(:, 1))
                    delete(this.edges(replot(i, 1)));
                    s = this.tree(replot(3), 1:3);
                    e = this.tree(replot(1), 1:3);
                    this.edges(replot(i, 1)) = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', 2, 'Color', 'r');
                end

            end

        end

    end

    methods (Static)

        function tmplot = display_searchline(s, e)
            tmplot = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', 3);
        end

        function tmplot = display_line(s, e)
            tmplot = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', 1, 'Color', 'b');
        end

    end

    methods (Access = public)

        function this = rrt(conf)
            this.maps = map(conf.filename);
            this.robot = uav(conf);
            X = this.maps.X;
            Y = this.maps.Y;
            Height = this.maps.Z;
            this.start = [X(conf.start(1)), Y(conf.start(2)), Height(conf.start(1), conf.start(2)) + conf.start(3), conf.start(4), conf.start(5), conf.start(6)];
            this.goal = [X(conf.goal(1)), Y(conf.goal(2)), Height(conf.goal(1), conf.goal(2)) + conf.goal(3), conf.goal(4), conf.goal(5), conf.goal(6)];
            this.threshold = conf.threshold;
            this.maxFailedAttempts = conf.maxFailedAttempts;
            %this.searchSize = conf.search * [(goal(1) - start(1)), (goal(2) - start(2)), goal(3) - start(3), 0, 0, 0];
            this.randnum = conf.randnum;
            this.failedAttempts = 0;
            this.nodenum = 1;
            this.searchbBase = [min(X), min(Y), min(min(Height)), -pi, -pi, -pi];
            this.searchSize = [max(X) - min(X), max(Y) - min(Y), max(max(Height)) - min(min(Height)), 2 * pi, 2 * pi, 2 * pi];

            figure(1)
            this.maps.display_map()
            scatter3(this.start(1), this.start(2), this.start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
            scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
            text(this.start(1), this.start(2), this.start(3), '  起点');
            text(this.goal(1), this.goal(2), this.goal(3), '  终点');
            xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
            title('RRT算法UAV航迹规划路径');

        end

        function starts(this, ifdispaly)
            this.tree = zeros(100, 6);

            this.newNode = this.start;
            this.insert_node(-1);

            tic

            while this.failedAttempts <= this.maxFailedAttempts
                sample = this.get_sample();
                %this.neighbors(sample, 100000);
                [closestNode, parentid] = this.get_closest(sample);
                this.extends(sample, closestNode);

                if ifdispaly
                    tp = this.display_searchline(closestNode, sample);
                    pause(0.05);
                    delete(tp);
                end

                flag = this.check_newNode(closestNode);

                if flag == 0
                    this.insert_node(parentid);

                elseif flag == 2

                    this.trace_back(parentid);
                    break;
                end

                if ifdispaly
                    this.display_line(closestNode, this.newNode);
                    pause(0.05);
                end

            end

            toc
            % t1 = saves('output', 'path', 0);
            % t2 = saves('output', 'RRTree', 1);
            % saveas(1, t1);
            % save(t2, 'RRTree');

        end

        function start_star(this, ifdispaly)
            this.tree = zeros(30000, 10); 
            this.edges = matlab.graphics.chart.primitive.Line(30000);
            this.newNode = this.start;
            this.newNode(7) = 0;
            this.insert_node(-1);
            neigh = (2 * this.robot.v * this.robot.deltaT) ^ 2;
            tic
            numb = 0;
            tooclose = 0;
            isgoal = 0;

            while toc <= 7 %this.failedAttempts <= this.maxFailedAttempts
                numb = numb + 1;
                sample = this.get_sample();
                [closestNode, parentid] = this.get_closest(sample);
                this.extends(sample, closestNode);

                if ifdispaly
                    tp = this.display_searchline(closestNode, sample);
                    % pause(0.05);
                    delete(tp);
                end

                if this.neighbors(neigh)
                    tooclose = tooclose + 1;

                    continue
                end

                flag = this.check_newNode();
                % this.neighbors(neigh);
                % new_id = this.choose_parent(closestNode);
                % this.insert_node(new_id);
                % replot = this.rewire(new_id);
                if flag == 0

                    new_id = this.choose_parent(closestNode);

                    if new_id > 0
                        this.insert_node(new_id);
                        replot = this.rewire(new_id);

                        if ifdispaly
                            this.edges(this.newNode(9)) = this.display_line(this.tree(this.newNode(8), :), this.newNode);
                            this.redisplay(replot);
                            % pause(0.05);
                        end

                    end

                elseif flag == 2
                    isgoal = isgoal + 1;
                    %this.randnum = 2;
                    this.trace_back(parentid);
                    %break;
                end

            end

            numb
            tooclose
            isgoal

        end

    end

end
