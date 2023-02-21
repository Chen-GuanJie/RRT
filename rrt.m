classdef rrt < handle

    properties (SetAccess = private)
        tree
        maps
        robot
        start
        goal
        threshold
        maxFailedAttempts
        searchSize
        searchbBase
        randnum

        nearNodes
        distances
        newNode
        failedAttempts
        nodenum
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

        function neighbors(this, sample, dis)
            %找附近的点
            t = this.tree(1:this.nodenum - 1, 1:6) - sample;
            this.distances = sqrt(sum(t(:, 1:6) .^ 2, 2));
            this.nearNodes = this.tree(find(this.distances < dis), :);
        end

        function [node, index] = get_closest(this)
            %找最近点
            [~, I] = min(this.distances);
            index = I(1);
            node = this.tree(index, :);
        end

        function extends(this, sample, closestNode)
            %延申
            this.newNode = this.robot.transfer(sample, closestNode, this.maps);
        end

        function cost = calc_cost(this, from_node, dest_node)
            %计算相邻两点的代价
            cost_dist = norm(from_node(1:3) - dest_node(1:3));
            cost_angle = norm(from_node(4:6) - dest_node(4:6));
            consumption = this.robot.calc_consumption();
            cost = norm([cost_dist, cost_angle, consumption]);
        end

        function flag = check_newNode(this, closestNode)
            flag = 0;

            if ~this.maps.checkPath(closestNode, this.newNode)
                this.failedAttempts = this.failedAttempts + 1;
                % RRTree(I(1), 8) = RRTree(I(1), 8) + 1;
                % RRTree(I(1), 3) = RRTree(I(1), 3) * 1.1;
                flag = 1;

            elseif norm(this.newNode(1:3) - this.goal(1:3)) < 2 * this.threshold
                flag = 2;
            end

            % 如果newPoint与之前RRTree上某一点的距离小于threshold说明newPoint的意义不大，舍弃
            [~, index] = min(norm(this.tree(:, 1:6) - this.newNode(1:6)), [], 1);

            if norm(this.tree(index(1), 1:6) - this.newNode(1:6)) < 1 * this.threshold
                this.failedAttempts = this.failedAttempts + 1;
                flag = 1;

            end

        end

        function insert_node(this, parent_id)
            %插入节点
            this.tree(this.nodenum, 1:6) = this.newNode;
            this.tree(this.nodenum,8) = parent_id;
            this.nodenum = this.nodenum + 1;
        end

        function trace_back(this, id)
            %回溯轨迹
            path = this.goal;
            prev = id;

            while prev > 0
                path = [this.tree(prev, 1:6); path];
                prev = this.tree(prev,8);
            end

            bar3 = plot3(path(:, 1), path(:, 2), path(:, 3), 'LineWidth', 2, 'color', 'r');

        end

        function new_parent_id = choose_parent(this, closestNode)
            %RRT star 找新的父节点
            mini_cost = this.calc_cost(closestNode, this.newNode);
            new_parent_id = 0;

            for i = 1:numel(this.nearNodes)

                if (~this.maps.checkPath(this.newNode, this.nearNodes(i)))
                    tmpcost = this.calc_cost(this.nearNodes(i), this.newNode);

                    if tmpcost < mini_cost
                        mini_cost = tmpcost;
                        new_parent_id = i;
                    end

                end

            end

        end

        function rewire(this)

            for i = 1:numel(this.nearNodes)

                if new_parent_id ~= i
                    tmpcost = this.calc_cost(this.newNode, this.nearNodes(i));

                    if tmpcost < this.nearNodes(i, 7)
                        this.nearNodes(i, 7)=tmpcost;

                    end

                end

            end

        end

    end

    methods (Access = public)

        function tmplot = display_searchline(~, s, e)
            tmplot = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', 3);
        end

        function tmplot = display_line(~, s, e)
            tmplot = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', 1);
        end

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
            this.searchbBase = [min(X), min(Y), min(min(Height)), -3.1416, -3.1416, -3.1416];
            this.searchSize = [max(X) - min(X), max(Y) - min(Y), max(max(Height)) - min(min(Height)), 2 * 3.1416, 2 * 3.1416, 2 * 3.1416];

            this.tree = zeros(100, 8);

        end

        function starts(this, ifdispaly)

            figure(1)
            this.maps.display_map()
            scatter3(this.start(1), this.start(2), this.start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
            scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
            text(this.start(1), this.start(2), this.start(3), '  起点');
            text(this.goal(1), this.goal(2), this.goal(3), '  终点');
            xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
            title('RRT算法UAV航迹规划路径');
            this.newNode = this.start;
            this.insert_node(-1);

            tic

            while this.failedAttempts <= this.maxFailedAttempts
                sample = this.get_sample();
                this.neighbors(sample, 100000);
                [closestNode, parentid] = this.get_closest();
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

            figure(1)
            this.maps.display_map()
            scatter3(this.start(1), this.start(2), this.start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
            scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
            text(this.start(1), this.start(2), this.start(3), '  起点');
            text(this.goal(1), this.goal(2), this.goal(3), '  终点');
            xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
            title('RRT算法UAV航迹规划路径');
            this.newNode = this.start;
            this.insert_node(-1);

            tic

            while this.failedAttempts <= this.maxFailedAttempts
                sample = this.get_sample();
                this.neighbors(sample, 100000);
                [closestNode, parentid] = this.get_closest();
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

    end

end
