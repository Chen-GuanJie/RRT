classdef rrt < handle

    properties (SetAccess = private)
        tree %节点位置 % cost parentid id cost_channge_to_parent cost_to_newNode
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
        randnums
        edges
        % multi_state
        % path_id = []
        height_cost_rate
        plot_point = []

        nearNodes
        newNode
        nodenum
        map_scale
        ifdisplay = false
        %informed
        informed = false
        long_axis % 长轴
        short_axis %短轴
        ellipse_rotation
        ellipse_displace
        %temp value
        compare_table
        tmp_ind
        tmp_value
        replot
        replot_num
        path_plot
        %output
        search_num = 0
        tooclose = 0
        isgoal = 0
        no_parent = 0
        collision = 0
        rewire_num = 0
        path
        path_sample
    end

    methods (Access = private)

        function sample = get_sample(this)
            %采样
            if rand < this.randnum

                if this.informed
                    y = 0;
                    sample = [0 inf 0 0 0 0]';

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
            this.compare_table = this.tree(1:this.nodenum - 1, 1:3) - this.newNode(1:3);
            this.compare_table = sum(this.compare_table(:, 1:3) .^ 2, 2);
            [this.tmp_value, ~] = min(this.compare_table); %最近的距离
            % this.tmp_value = sqrt(this.tmp_value); %最近的距离

            if this.tmp_value < this.threshold_close %太近
                flag = 1;
            end

            this.nearNodes = this.tree(this.compare_table < dis, :); %todo: if use find
        end

        function [node, index] = get_closest(this, sample)
            %找最近点
            this.compare_table = this.tree(1:this.nodenum - 1, 1:3) - sample(1:3);
            [~, index] = min(sum(this.compare_table(:, 1:3) .^ 2, 2));
            node = this.tree(index, :);
        end

        function extends(this, sample, closestNode)
            %延申
            this.newNode = this.robot.transfer(sample, closestNode, this.maps);
        end

        function cost = calc_cost(~, from_node, dest_node)
            %计算相邻两点的代价
            cost_dist = norm(from_node(1:2) - dest_node(1:2));
            cost_hight = 6 * norm(from_node(3) - dest_node(3));
            cost_angle = norm(from_node(4:6) - dest_node(4:6));
            %consumption = this.robot.calc_consumption();
            cost = norm([cost_dist, cost_angle, cost_hight]);
        end

        function [id, flag] = get_ancestor(this, prev)
            flag = false;
            id = zeros(1, 1);
            this.tmp_ind = 1;
            prev = this.tree(prev, 8);

            while prev > 0
                flag = true;
                id(this.tmp_ind, 1) = prev;
                this.tmp_ind = this.tmp_ind + 1;
                % this.path(this.tmp_ind, :) = this.tree(prev, :);
                prev = this.tree(prev, 8);
            end

        end

        function output = cumcost(this, prev)
            this.tmp_ind = 1;
            output = 0;

            while prev > 0
                output = output + this.tree(prev, 10);
                prev = this.tree(prev, 8);
            end

        end

        function cost = calc_cost_v2(this, from_node, dest_node, new_target_h)
            cost_dist = norm(from_node(1:2) - dest_node(1:2));
            cost_hight = this.height_cost_rate * sum(abs(diff(new_target_h)));
            % cost_angle = norm(from_node(4:6) - dest_node(4:6));
            %consumption = this.robot.calc_consumption();
            cost = norm([cost_dist, cost_hight]);

        end

        function flag = check_newNode(this)
            flag = 0;

            if norm(this.newNode(1:3) - this.goal(1:3)) < this.threshold_goal
                flag = 2;
            end

        end

        function flag = collisionCheck(this, from, new)

            % retval=this.maps.checkPath(from, new);

            flag = 1;

            if ~this.maps.checkPath(from, new) %todo:  旧的判断
                this.tree(from(9), 3) = this.tree(from(9), 3) * 1.02;
                flag = 0;
            end

        end

        function insert_node(this, parent_id)
            %插入节点
            this.newNode(1, 8:9) = [parent_id this.nodenum];
            this.tree(this.nodenum, 1:10) = this.newNode;
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

            path_num = this.tmp_ind;
            this.tmp_value = this.path(2:this.tmp_ind, 1:3) - this.path(1:this.tmp_ind - 1, 1:3);
            path_len = sum(sqrt(sum(this.tmp_value .^ 2, 2)));

            if this.ifdisplay

                if ~isempty(this.path_plot)
                    delete(this.path_plot);
                end

                this.path_plot = plot3(this.path(1:this.tmp_ind, 1), this.path(1:this.tmp_ind, 2), this.path(1:this.tmp_ind, 3), 'LineWidth', 2, 'color', 'g');
            end

        end

        function path_num = find_best_path(this, path_id)
            [~, path_number] = size(path_id);
            path_length = zeros(path_number, 1);
            this.ifdisplay = true;

            for i = 1:path_number
                [path_length(i, 1), ~] = this.trace_back(path_id(i));
            end

            [~, best_path_id] = min(path_length);
            [~, path_num] = this.trace_back(path_id(best_path_id));
        end

        function new_parent_id = choose_parent_v2(this)

            for i = 1:numel(this.nearNodes(:, 1))
                [target_h, delta_dist, flag] = this.maps.checkPath_v2(this.nearNodes(i, :), this.newNode);

                if flag
                    [new_target_h, flag2] = this.robot.follow(this.nearNodes(i, :), this.newNode, target_h(:, 3), delta_dist);

                    if flag2
                        this.tmp_value = this.calc_cost_v2(this.nearNodes(i, :), this.newNode, new_target_h); % 新节点与附近的相邻转移代价 + this.nearNodes(i, 7);
                        this.nearNodes(i, 7) = this.cumcost(this.nearNodes(i, 9)); %this.nearNodes(i, 7) + delta_cost;
                    else
                        this.tmp_value = inf;
                    end

                else
                    this.tmp_value = inf;
                end

                this.nearNodes(i, 11) = this.tmp_value;
            end

            this.compare_table = this.nearNodes(:, 11) + this.nearNodes(:, 7); %以附近点作新节点父亲后的代价
            [this.newNode(1, 7), this.tmp_ind] = min(this.compare_table);
            this.newNode(1, 10) = this.nearNodes(this.tmp_ind, 11);
            new_parent_id = this.nearNodes(this.tmp_ind, 9);

        end

        function new_parent_id = choose_parent(this)
            %RRT star 找新的父节点
            mini_cost = Inf;
            new_parent_id = 0;

            this.tmp_ind = 1;
            n = numel(this.nearNodes(:, 1));

            while this.tmp_ind <= n

                if (this.collisionCheck(this.nearNodes(this.tmp_ind, :), this.newNode)) %无碰撞

                    if (this.robot.transferable(this.nearNodes(this.tmp_ind, :), this.newNode)) %可转移

                        this.tmp_value = this.calc_cost(this.nearNodes(this.tmp_ind, :), this.newNode) + this.nearNodes(this.tmp_ind, 7);

                        if this.tmp_value < mini_cost
                            mini_cost = this.tmp_value;
                            new_parent_id = this.nearNodes(this.tmp_ind, 9);
                        end

                    end

                    this.tmp_ind = this.tmp_ind + 1;
                else %删去集合中有碰撞的附近点
                    this.nearNodes(this.tmp_ind, :) = [];
                    this.tmp_ind = this.tmp_ind - 1;
                    n = n - 1;

                    if this.tmp_ind == 0
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
                        this.tmp_value = this.calc_cost(this.newNode, this.nearNodes(i, :)) + this.newNode(7);

                        if this.tmp_value < this.nearNodes(i, 7)
                            % id parentid newparentid
                            this.replot_num = this.replot_num + 1;
                            this.replot (this.replot_num, :) = [this.nearNodes(i, 9) this.nearNodes(i, 8) this.newNode(9)];
                            this.nearNodes(i, 7) = this.tmp_value;
                            this.nearNodes(i, 8) = this.newNode(9);
                        end

                    end

                end

            end

        end

        function rewire_v2(this, new_parent_id)
            %重布线
            this.nearNodes(this.nearNodes(:, 9) == new_parent_id, :) = [];
            this.compare_table = this.nearNodes(:, 11) + this.newNode(7); %新节点作为附近点的父节点后的代价
            index = (this.compare_table(:, 1) - this.nearNodes(:, 7)) < 0; %替换后代价变小的点的下标
            this.tmp_ind = this.nearNodes(index, 9); %替换后代价变小的点的id
            [this.replot_num, ~] = size(this.tmp_ind);
            this.rewire_num = this.rewire_num + this.replot_num;

            if this.ifdisplay
                this.replot(1:this.replot_num, 1:2) = [this.tmp_ind this.tree(this.tmp_ind, 8)];
                this.replot(1:this.replot_num, 3) = this.newNode(9);
            end

            this.tree(this.tmp_ind, 10) = this.nearNodes(index, 11);
            this.tree(this.tmp_ind, 8) = this.newNode(9);

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

        function s = interpolation(this, path_num)
            interp_num = 0.1;
            tmp = interp1(1:path_num, this.path(1:path_num, 1:6), 1:interp_num:path_num, 'spline');
            [s, ~] = size(tmp);
            this.path(1:s, 1:6) = tmp;
            this.path(s, 7) = 0;

            for i = s - 1:-1:1
                this.path(i, 7) = this.calc_cost(this.path(i + 1, :), this.path(i, :)) + this.path(i + 1, 7); %轨迹上的cost
            end

            for i = 1:s
                this.path(i, 9) = this.maps.Z(round(this.path(i, 1)), round(this.path(i, 2))); %轨迹对应的地形高度
                this.path(i, 8) = this.path(i, 3) - this.path(i, 9); %轨迹上的高度差
            end

            % this.path(:,1:3)=this.path(:,1:3)*this.map_scale;
            % this.path(:,8:9)=this.path(:,8:9)*this.map_scale;

            % for this.tmp_ind = 1:6
            %     this.path(1:interp_num:path_num,this.tmp_ind)=interp1(1:path_num,this.path(1:path_num,this.tmp_ind ),1:interp_num:path_num,'spline');
            % end

        end

        function [new_path, num] = follow_ground(this, path_num)
            new_path = zeros(400, 10);
            this.tmp_ind = 1;

            for i = path_num:-1:2
                [target_h, delta_dist] = this.maps.checkPath_v2(this.path(i, 1:3), this.path(i - 1, 1:3));
                new_target_h = this.robot.just_follow(target_h(:, 3), delta_dist);
                target_h(:, 3) = new_target_h;
                [this.tmp_value, ~] = size(target_h);
                % new_path(this.tmp_ind, :) = this.path(i, :);
                % new_path(this.tmp_ind, 9) = this.maps.find_height(this.path(i, :));

                new_path(this.tmp_ind:this.tmp_ind + this.tmp_value - 2, 1:3) = target_h(1:this.tmp_value - 1, 1:3);
                new_path(this.tmp_ind:this.tmp_ind + this.tmp_value - 2, 9) = target_h(1:this.tmp_value - 1, 4); %地形高度

                this.tmp_ind = this.tmp_ind + this.tmp_value;
            end

            % new_path(this.tmp_ind, :) = this.path(1, :);
            % new_path(this.tmp_ind, 9) = this.maps.find_height(this.path(1, :));

            new_path(:, 8) = new_path(:, 3) - new_path(:, 9); %轨迹上的高度差
            new_path = new_path(new_path(:, 3) ~= 0, :);
            % for i = 1:this.tmp_ind
            %     % new_path(i, 9) = this.maps.Z(y, round(new_path(i, 2))); %轨迹对应的地形高度
            %     new_path(i, 8) = new_path(i, 3) - new_path(i, 9); %轨迹上的高度差
            % end

            % num = this.tmp_ind - 1;
            [num, ~] = size(new_path);
        end

        function path_evaluate(this, path_num)
            figure(2)
            % subplot(5, 1, 1)
            % plot(1:path_num, this.path(path_num:-1:1, 4), 'LineWidth', 1.5, 'color', 'b', 'DisplayName', '航向角'); legend
            % subplot(5, 1, 2)
            % plot(1:path_num, this.path(path_num:-1:1, 5), 'LineWidth', 1.5, 'color', 'g', 'DisplayName', '滚转角'); legend
            % subplot(5, 1, 3)
            % plot(1:path_num, this.path(path_num:-1:1, 6), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '俯仰角'); legend
            % subplot(5, 1, 4)
            % plot(1:path_num, this.path(path_num:-1:1, 7), 'LineWidth', 1.5, 'color', 'k', 'DisplayName', 'cost'); legend
            % subplot(5, 1, 5)
            % plot(1:path_num, this.path(path_num:-1:1, 8), 'LineWidth', 1.5, 'color', 'k', 'DisplayName', '离地高度'); legend
            % distan = norm(this.path(1, 1:2) - this.path(path_num, 1:2)) * this.map_scale;
            % distan = 400 * distan / path_num;
            clear gca
            subplot(3, 1, 1)
            this.path(:, 1:3) = this.path(:, 1:3) * this.map_scale;
            this.path(:, 8:9) = this.path(:, 8:9) * this.map_scale;
            % this.path_sample(:, 1:3) = this.path_sample(:, 1:3) * this.map_scale;
            % this.path_sample(:, 8:9) = this.path_sample(:, 8:9) * this.map_scale;

            plot(1:path_num, this.path(path_num:-1:1, 3), 'LineWidth', 1.5, 'color', 'b', 'DisplayName', '飞机高度'); hold on
            % set(gca, 'Xtick', [0  1000 distan]);
            plot(1:path_num, this.path(path_num:-1:1, 9), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '地形高度');

            legend

            subplot(3, 1, 2)
            % [n,~]=size(this.path_sample);
            % plot(1:n, this.path_sample(n:-1:1, 9), 'LineWidth', 1.5, 'color', 'b', 'DisplayName', '飞机高度');hold on
            plot(1:path_num, this.path(path_num:-1:1, 9), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '地形高度');
            % set(gca, 'Xtick', [0 1000 distan]);

            legend

            subplot(3, 1, 3)
            plot(1:path_num, this.path(path_num:-1:1, 8), 'LineWidth', 1.5, 'color', 'k', 'DisplayName', '离地高度');
            % set(gca, 'Xtick', [0 50000 distan]);
            legend

        end

        function display_serachline(this, closestNode, sample, delay_time)
            tp = this.display_line(closestNode, sample, 3, 'k');

            if delay_time ~= 0
                pause(delay_time);
            end

            delete(tp);

        end

        function display_states(this, delay_time)
            % this.display_arrow(this.newNode, 10); %
            this.edges(this.newNode(9)) = this.display_line(this.tree(this.newNode(8), :), this.newNode, 1, 'b');
            this.redisplay();

            if delay_time ~= 0
                pause(delay_time);
            end

        end

        function output = to_normal_size(this)
            this.path(:, 1:3) = this.path(:, 1:3) * this.map_scale;
            this.path(:, 1) = this.path(:, 1) + this.maps.X(1);
            this.path(:, 2) = this.path(:, 2) + this.maps.Y(1);
            output = this.path(:, 1:3);
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

        function output = start_star(this, ifdisplay, max_time, delay_time)

            if nargin < 3
                ifdisplay = false;
                max_time = 7;
                delay_time = 0.01;
            end

            this.tree = zeros(this.max_nodes, 10);
            % this.multi_state = zeros(this.max_nodes, 4);
            this.path = zeros(500, 10);
            this.ifdisplay = ifdisplay;
            this.nodenum = 1;
            this.informed = false;
            this.randnum = this.randnums(1, 1);
            this.search_num = 0;
            this.tooclose = 0;
            this.isgoal = 0;
            this.no_parent = 0;
            this.rewire_num = 0;
            figure(1);

            if this.ifdisplay

                if ~isempty(this.edges)
                    delete(this.edges);
                end

                this.edges = matlab.graphics.chart.primitive.Line(this.max_nodes);
            end

            path_id = zeros(5, 1);
            this.newNode = [this.start 0 0 0 0];
            this.insert_node(-1);
            neighbor_dist = 20 ^ 2; %(2 * this.robot.v * this.robot.deltaT) ^ 2;
            tic
            % output = zeros(1, 1);
            % ind = 1;
            % last = 0;

            while toc <= max_time
                this.search_num = this.search_num + 1;
                sample = this.get_sample();
                [closestNode, ~] = this.get_closest(sample);
                this.newNode = this.robot.transfer_directly(sample, closestNode, this.maps.Z);

                if this.ifdisplay
                    this.display_serachline(closestNode, sample, delay_time);
                end

                if this.neighbors(neighbor_dist)
                    this.tooclose = this.tooclose + 1;
                    continue
                end

                new_id = this.choose_parent_v2();

                if norm(this.newNode(1:3) - this.goal(1:3)) > this.threshold_goal

                    if new_id > 0
                        this.insert_node(new_id);
                        this.rewire_v2(new_id);

                        if this.ifdisplay
                            this.display_states(delay_time)
                        end

                    else
                        this.no_parent = this.no_parent + 1;
                    end

                else %if flag == 2
                    this.isgoal = this.isgoal + 1;
                    path_id(this.isgoal, 1) = new_id;
                    this.randnum = this.randnums(1, 2); %搜索点不取goal
                    [path_len, ~] = this.trace_back(new_id);
                    this.prepare_informed(path_len);
                end

                % if toc - last > 0.5
                %     last = toc;
                %     output(ind) = this.search_num;
                %     ind = ind + 1;
                % end

            end

            % [~, path_number] = size(this.path_id);
            path_num = this.find_best_path(path_id);
            fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d', this.search_num, this.tooclose, this.isgoal, this.no_parent, this.rewire_num);
            % interp_num = this.interpolation(path_num);
            [new_path, interp_num] = this.follow_ground(path_num);

            if ~isempty(this.path_plot)
                delete(this.path_plot);
            end

            this.path = new_path;
            plot3(this.path(1:interp_num, 1), this.path(1:interp_num, 2), this.path(1:interp_num, 3), 'LineWidth', 2, 'color', 'g');
            % this.path_sample=this.path;
            this.path_evaluate(interp_num);
            output = this.maps.to_normal_size(this.path);
        end

        function start_rrt(this, ifdisplay, max_time, delay_time)
            this.tree = zeros(this.max_nodes, 6);
            this.path = zeros(100, 6);
            this.newNode = this.start;
            this.insert_node(-1);
            this.ifdisplay = ifdisplay;
            tic

            while toc < max_time
                sample = this.get_sample();
                [closestNode, parentid] = this.get_closest(sample);
                this.extends(sample, closestNode);

                if this.ifdisplay
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

                if this.ifdisplay
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

        function set_start_end(this, s, g)
            s(3) = this.maps.Z(s(1), s(2)) + s(3) / this.map_scale;
            g(3) = this.maps.Z(g(1), g(2)) + g(3) / this.map_scale;

            this.start = s;
            this.goal = g;
            figure(1)
            delete(this.plot_point(1).point);
            delete(this.plot_point(2).point);
            delete(this.plot_point(1).text);
            delete(this.plot_point(2).text);

            this.plot_point(1).point = scatter3(this.start(1), this.start(2), this.start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k');
            this.plot_point(1).text = text(this.start(1), this.start(2), this.start(3), '  起点');
            this.plot_point(2).point = scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', 'o', 'MarkerEdgeColor', 'k');
            this.plot_point(2).text = text(this.goal(1), this.goal(2), this.goal(3), '  终点');

        end

        function set_params(this, conf)
            this.height_cost_rate = conf.height_cost_rate;
            this.threshold_close = (conf.threshold_close / this.map_scale) ^ 2;
            this.threshold_goal = conf.threshold_goal / this.map_scale;
            this.max_nodes = conf.max_nodes;
            this.randnums = conf.randnum;
        end

        function this = rrt(conf)
            this.maps = map(conf.filename);
            conf.map_scale = this.maps.X(2) - this.maps.X(1);
            this.map_scale = conf.map_scale;
            this.maps.set_height_limit(conf.height_limit / this.map_scale);
            Height = this.maps.Z;
            this.start = [conf.start(1), conf.start(2), Height(conf.start(1), conf.start(2)) + conf.start(3) / this.map_scale, conf.start(4), conf.start(5), conf.start(6)];
            this.goal = [conf.goal(1), conf.goal(2), Height(conf.goal(1), conf.goal(2)) + conf.goal(3) / this.map_scale, conf.goal(4), conf.goal(5), conf.goal(6)];
            this.searchbBase = [1, 1, min(min(Height)), -pi, -pi, -pi];
            this.searchSize = [this.maps.X_num - 1, this.maps.Y_num - 1, max(max(Height)) - min(min(Height)), 2 * pi, 2 * pi, 2 * pi];
            this.replot = zeros(50, 3);
            this.robot = uav(conf);
            this.set_params(conf);
            figure(1)
            this.maps.display_map()
            this.plot_point(1).point = scatter3(this.start(1), this.start(2), this.start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
            this.plot_point(2).point = scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
            this.plot_point(1).text = text(this.start(1), this.start(2), this.start(3), '  起点');
            this.plot_point(2).text = text(this.goal(1), this.goal(2), this.goal(3), '  终点');
            xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
            title('RRT算法UAV航迹规划路径');

        end

    end

end
