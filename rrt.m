classdef rrt < handle

    properties (SetAccess = private)
        tree %节点位置 % cost parentid id v
        time_cost %时间消耗
        energe_cost %能量消耗
        velocity %速度
        max_nodes
        maps
        robot
        start
        goal
        threshold_close
        threshold_goal
        searchSize
        searchbBase
        randnum
        edges
        multi_state

        nearNodes
        distances
        newNode
        nodenum
        map_scale

        informed
        long_axis % 长轴
        short_axis %短轴
        ellipse_rotation
        ellipse_displace
        %temp value
        compare_table
        tmp_ind
        replot
        replot_num
        path_plot
        %output
        search_num
        tooclose
        isgoal
        no_parent
        collision
        path
    end

    methods (Access = private)

        function sample = get_sample(this)
            %采样
            sample = [0 inf 0 0 0 0]';

            if rand < this.randnum

                if this.informed
                    y = 0;

                    while abs(sample(2)) > y
                        sample(1:2) = [rand * 2 * this.long_axis; rand * 2 * this.short_axis] - [this.long_axis; this.short_axis];
                        y = this.short_axis * sqrt(1 - (sample(1) / this.long_axis) ^ 2);
                    end

                    sample(3) = rand * this.searchSize(3) + this.searchbBase(3);
                    sample(1:2) = this.ellipse_rotation * sample(1:2) + this.ellipse_displace;
                    sample = sample';
                else

                    sample = rand(1, 6) .* this.searchSize + this.searchbBase;

                end

            else
                sample = this.goal;
            end

        end

        function flag = neighbors(this, dis)
            %找附近的点
            flag = 0;
            this.compare_table = this.tree(1:this.nodenum - 1, 1:6) - this.newNode(1:6);
            this.distances = sum(this.compare_table(:, 1:3) .^ 2, 2);
            [~, index] = min(this.distances);
            tmpdis = sqrt(this.distances(index(1)));

            if tmpdis < this.threshold_close %太近
                flag = 1;
            end

            this.nearNodes = this.tree(find(this.distances < dis), :);
        end

        function [node, index] = get_closest(this, sample)
            %找最近点
            this.compare_table = this.tree(1:this.nodenum - 1, 1:3) - sample(1:3);
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

            if norm(this.newNode(1:3) - this.goal(1:3)) < this.threshold_goal
                flag = 2;
            end

        end

        function flag = collisionCheck(this, from, new)
            flag = 1;

            if ~this.maps.checkPath(from, new)
                this.tree(from(9), 3) = this.tree(from(9), 3) * 1.02;
                flag = 0;
            end

        end

        function insert_node(this, parent_id)
            %插入节点
            this.newNode(8) = parent_id;
            this.newNode(9) = this.nodenum;
            this.tree(this.nodenum, 1:9) = this.newNode;
            this.nodenum = this.nodenum + 1;
        end

        function [path_len, path_num] = trace_back(this, id)
            %回溯轨迹
            this.tmp_ind = 1;
            this.path(this.tmp_ind, :) = [this.goal 0 0 0 0];
            prev = id;

            while prev > 0
                this.tmp_ind = this.tmp_ind + 1;
                this.path(this.tmp_ind, :) = this.tree(prev, :);
                prev = this.tree(prev, 8);
            end

            tmp = this.path(2:this.tmp_ind, 1:3) - this.path(1:this.tmp_ind - 1, 1:3);
            path_len = sum(sqrt(sum(tmp .^ 2, 2)));

            if ~isempty(this.path_plot)
                delete(this.path_plot);
            end

            path_num = this.tmp_ind;
            this.path_plot = plot3(this.path(1:this.tmp_ind, 1), this.path(1:this.tmp_ind, 2), this.path(1:this.tmp_ind, 3), 'LineWidth', 2, 'color', 'g');

        end

        function new_parent_id = choose_parent(this)
            %RRT star 找新的父节点
            mini_cost = Inf;
            new_parent_id = 0;

            i = 1;
            n = numel(this.nearNodes(:, 1));

            while i <= n

                if (this.collisionCheck(this.nearNodes(i, :), this.newNode)) %无碰撞

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

                    if i == 0
                        break;
                    end

                end

            end

            this.newNode(7) = mini_cost;

        end

        function rewire(this, new_parent_id)
            %重布线
            this.replot_num = 0;

            for i = 1:numel(this.nearNodes(:, 1))

                if new_parent_id ~= this.nearNodes(i, 8) %不是新节点的父节点

                    if this.robot.transferable(this.newNode, this.nearNodes(i, :))
                        tmpcost = this.calc_cost(this.newNode, this.nearNodes(i, :)) + this.newNode(7);

                        if tmpcost < this.nearNodes(i, 7)
                            % id parentid newparentid
                            this.replot_num = this.replot_num + 1;
                            this.replot (this.replot_num, :) = [this.nearNodes(i, 9) this.nearNodes(i, 8) this.newNode(9)];
                            this.nearNodes(i, 7) = tmpcost;
                            this.nearNodes(i, 8) = this.newNode(9);
                        end

                    end

                end

            end

        end

        function redisplay(this)

            for i = 1:this.replot_num
                delete(this.edges(this.replot(i, 1)));
                s = this.tree(this.replot(i, 3), 1:3);
                e = this.tree(this.replot(i, 1), 1:3);
                this.edges(this.replot(i, 1)) = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', 1, 'Color', 'r');
            end

        end

        function prepare_informed(this, path_len)
            this.informed = true; %开始informed搜索
            dist = norm(this.start(1:3) - this.goal(1:3)) / 2;

            this.long_axis = path_len / 2; % 长轴 %1.05 * dist;
            this.short_axis = sqrt(this.long_axis ^ 2 - dist ^ 2); %短轴
            ang = atan2(this.goal(2) - this.start(2), this.goal(1) - this.start(1));
            this.ellipse_rotation = [cos(ang) -sin(ang); sin(ang) cos(ang)];
            this.ellipse_displace = [dist * cos(ang); dist * sin(ang)] + [this.start(1); this.start(2)];

        end

        function path_evaluate(this, path_num)
            figure(2)
            subplot(4, 1, 1)
            plot(1:path_num, this.path(path_num:-1:1, 4), 'LineWidth', 1.5, 'color', 'b', 'DisplayName', '航向角'); legend
            subplot(4, 1, 2)
            plot(1:path_num, this.path(path_num:-1:1, 5), 'LineWidth', 1.5, 'color', 'g', 'DisplayName', '滚转角'); legend
            subplot(4, 1, 3)
            plot(1:path_num, this.path(path_num:-1:1, 6), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '俯仰角'); legend
            subplot(4, 1, 4)
            plot(1:path_num, this.path(path_num:-1:1, 7), 'LineWidth', 1.5, 'color', 'k', 'DisplayName', 'cost'); legend

        end

    end

    methods (Static)

        function tmplot = display_line(s, e, LineWidth, color)
            tmplot = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', LineWidth, 'Color', color);
        end

        function tmplot = display_arrow(p, len)
            tmplot = quiver3(p(1), p(2), p(3), len * cos(p(4)), len * sin(p(4)), len * sin(p(6)), 'LineWidth', 1.5, 'MaxHeadSize', 50);
        end

    end

    methods (Access = public)

        function start_star(this, ifdispaly, max_time, delay_time)
            this.tree = zeros(this.max_nodes, 10);
            this.multi_state = zeros(this.max_nodes, 4);
            this.path = zeros(100, 10);
            this.edges = matlab.graphics.chart.primitive.Line(this.max_nodes);
            this.newNode = [this.start 0];
            this.insert_node(-1);
            neigh = 20 ^ 2; %(2 * this.robot.v * this.robot.deltaT) ^ 2;

            tic
            this.search_num = 0;
            this.tooclose = 0;
            this.isgoal = 0;
            this.no_parent = 0;
            this.collision = 0;

            while toc <= max_time
                this.search_num = this.search_num + 1;
                sample = this.get_sample();
                [closestNode, parentid] = this.get_closest(sample);
                this.extends(sample, closestNode);

                if ifdispaly
                    tp = this.display_line(closestNode, sample, 3, 'k');

                    if delay_time ~= 0
                        pause(delay_time);
                    end

                    delete(tp);

                end

                if this.neighbors(neigh)
                    this.tooclose = this.tooclose + 1;
                    continue
                end

                flag = this.check_newNode();

                if flag == 0

                    new_id = this.choose_parent();

                    if new_id > 0
                        this.insert_node(new_id);
                        this.rewire(new_id);

                        if ifdispaly

                            this.display_arrow(this.newNode, 10); %

                            this.edges(this.newNode(9)) = this.display_line(this.tree(this.newNode(8), :), this.newNode, 1, 'b');
                            this.redisplay();

                            if delay_time ~= 0
                                pause(delay_time);
                            end

                        end

                    else
                        this.no_parent = this.no_parent + 1;

                    end

                elseif flag == 2
                    this.isgoal = this.isgoal + 1;
                    this.randnum = 0.7; %搜索点不取goal
                    [path_len, path_num] = this.trace_back(parentid);
                    this.prepare_informed(path_len);
                end

            end

            this.search_num
            this.tooclose
            this.isgoal
            this.no_parent
            this.collision
            this.path_evaluate(path_num);
        end

        function start_rrt(this, ifdispaly, max_time, delay_time)
            this.tree = zeros(this.max_nodes, 6);
            this.path = zeros(100, 6);
            this.newNode = this.start;
            this.insert_node(-1);
            tic

            while toc < max_time
                sample = this.get_sample();
                [closestNode, parentid] = this.get_closest(sample);
                this.extends(sample, closestNode);

                if ifdispaly
                    tp = this.display_line(closestNode, sample, 3, 'k');

                    if delay_time ~= 0
                        pause(delay_time);
                    end

                    delete(tp);
                end

                flag = this.check_newNode();

                if flag == 0
                    this.insert_node(parentid);

                elseif flag == 2

                    this.trace_back(parentid);
                    break;
                end

                if ifdispaly
                    this.display_line(this.tree(this.newNode(8), :), this.newNode, 1, 'b');

                    if delay_time ~= 0
                        pause(delay_time);
                    end

                end

            end

            toc
            % t1 = saves('output', 'path', 0);
            % t2 = saves('output', 'RRTree', 1);
            % saveas(1, t1);
            % save(t2, 'RRTree');

        end

        function this = rrt(conf)
            this.maps = map(conf.filename);
            conf.map_scale = this.maps.X(2) - this.maps.X(1);
            this.map_scale = conf.map_scale;

            % X = this.maps.X;
            % Y = this.maps.Y;
            Height = this.maps.Z;
            % this.start = [X(conf.start(1)), Y(conf.start(2)), Height(conf.start(1), conf.start(2)) + conf.start(3), conf.start(4), conf.start(5), conf.start(6)];
            % this.goal = [X(conf.goal(1)), Y(conf.goal(2)), Height(conf.goal(1), conf.goal(2)) + conf.goal(3), conf.goal(4), conf.goal(5), conf.goal(6)];
            this.start = [conf.start(1), conf.start(2), Height(conf.start(1), conf.start(2)) + conf.start(3) / this.map_scale, conf.start(4), conf.start(5), conf.start(6)];
            this.goal = [conf.goal(1), conf.goal(2), Height(conf.goal(1), conf.goal(2)) + conf.goal(3) / this.map_scale, conf.goal(4), conf.goal(5), conf.goal(6)];

            % this.threshold = conf.threshold;
            %this.searchSize = conf.search * [(goal(1) - start(1)), (goal(2) - start(2)), goal(3) - start(3), 0, 0, 0];
            this.randnum = conf.randnum;
            this.nodenum = 1;
            % this.searchbBase = [min(X), min(Y), min(min(Height)), -pi, -pi, -pi];
            % this.searchSize = [max(X) - min(X), max(Y) - min(Y), max(max(Height)) - min(min(Height)), 2 * pi, 2 * pi, 2 * pi];

            this.searchbBase = [1, 1, min(min(Height)), -pi, -pi, -pi];
            this.searchSize = [this.maps.X_num - 1, this.maps.Y_num - 1, max(max(Height)) - min(min(Height)), 2 * pi, 2 * pi, 2 * pi];

            this.max_nodes = conf.max_nodes;
            this.informed = false;
            this.replot = zeros(10, 3);

            this.threshold_close = conf.threshold_close / conf.map_scale;
            this.threshold_goal = conf.threshold_goal / conf.map_scale;

            this.robot = uav(conf);

            figure(1)
            this.maps.display_map()
            scatter3(this.start(1), this.start(2), this.start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
            scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
            text(this.start(1), this.start(2), this.start(3), '  起点');
            text(this.goal(1), this.goal(2), this.goal(3), '  终点');
            xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
            title('RRT算法UAV航迹规划路径');

        end

    end

end
