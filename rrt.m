classdef rrt < handle

    properties (SetAccess = public)
        tree = zeros(100, 10) %节点位置 % cost parentid id cost_channge_to_parent cost_to_newNode
        max_nodes = 100
        maps %= map([0 0])
        robot %= uav()
        start = zeros(1, 6)
        goal = zeros(1, 6)
        threshold_close = 1
        threshold_goal = 1
        searchSize = zeros(1, 6)
        searchbBase = zeros(1, 6)
        randnum = 1
        randnums = zeros(1, 2)
        % multi_state
        height_cost_rate = 6

        nearNodes %= zeros(50, 11)
        newNode = zeros(1, 10)
        nodenum = 10
        height_scale = 500
        map_scale = 1
        %informed
        informed = false
        long_axis = 1 % 长轴
        short_axis = 1 %短轴
        ellipse_rotation = zeros(2, 2)
        ellipse_displace = zeros(2, 1)
        %temp value
        % compare_table = zeros(1, 1)
        % tmp_ind = zeros(10, 1)
        % tmp_value = 1
        replot_num = 1
        %output
        search_num = 0
        tooclose = 0
        isgoal = 0
        no_parent = 0
        collision = 0
        rewire_num = 0
        path = zeros(100, 10)
        path_sample = zeros(2, 2)
    end

    methods (Access = public)

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
            compare_table = zeros(this.nodenum - 1, 2);
            compare_table(:, 1:2) = this.tree(1:this.nodenum - 1, 1:2) - this.newNode(1:2);
            compare_table(:, 1) = sum(compare_table(:, 1:2) .^ 2, 2);
            [tmp_value, ~] = min(compare_table(:, 1)); %最近的距离
            % tmp_value = sqrt(tmp_value); %最近的距离

            if tmp_value < this.threshold_close %太近
                flag = 1;
            end

            compare_table = find(compare_table(:, 1) < dis);
            this.nearNodes = zeros(length(compare_table), 11);
            this.nearNodes(:, 1:10) = this.tree(compare_table, :);
            this.nearNodes(:, 11) = 0;
        end

        function [node, index] = get_closest(this, sample)
            %找最近点
            compare_table = this.tree(1:this.nodenum - 1, 1:2) - sample(1:2);
            [~, index] = min(sum(compare_table(:, 1:2) .^ 2, 2));
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
            ind = 1;
            prev = this.tree(prev, 8);

            while prev > 0
                flag = true;
                id(ind, 1) = prev;
                ind = ind + 1;
                % this.path(ind, :) = this.tree(prev, :);
                prev = this.tree(prev, 8);
            end

        end

        function output = cumcost(this, prev)
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
            ind = 1;
            this.path(ind, :) = [this.goal 0 0 0 0];
            prev = id;

            while prev > 0
                ind = ind + 1;
                this.path(ind, :) = this.tree(prev, :);
                prev = this.tree(prev, 8);
            end

            path_num = ind;
            tmp_value = zeros(ind - 1, 1);
            tmp_value(:, 1:3) = this.path(2:ind, 1:3) - this.path(1:ind - 1, 1:3);
            path_len = sum(sqrt(sum(tmp_value .^ 2, 2)));

        end

        function path_num = find_best_path(this, path_id)
            [~, path_number] = size(path_id);
            path_length = zeros(path_number, 1);

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
                        tmp_value = this.calc_cost_v2(this.nearNodes(i, :), this.newNode, new_target_h); % 新节点与附近的相邻转移代价 + this.nearNodes(i, 7);
                        this.nearNodes(i, 7) = this.cumcost(this.nearNodes(i, 9)); %this.nearNodes(i, 7) + delta_cost;
                    else
                        tmp_value = inf;
                    end

                else
                    tmp_value = inf;
                end

                this.nearNodes(i, 11) = tmp_value;
            end

            % compare_table = zeros(length(this.nearNodes(:, 1)), 1);
            compare_table = this.nearNodes(:, 11) + this.nearNodes(:, 7); %以附近点作新节点父亲后的代价
            [this.newNode(1, 7), ind] = min(compare_table(:, 1));
            this.newNode(1, 10) = this.nearNodes(ind, 11);
            new_parent_id = this.nearNodes(ind, 9);

        end

        function new_parent_id = choose_parent(this)
            %RRT star 找新的父节点
            mini_cost = Inf;
            new_parent_id = 0;

            ind = 1;
            n = numel(this.nearNodes(:, 1));

            while ind <= n

                if (this.collisionCheck(this.nearNodes(ind, :), this.newNode)) %无碰撞

                    if (this.robot.transferable(this.nearNodes(ind, :), this.newNode)) %可转移

                        tmp_value = this.calc_cost(this.nearNodes(ind, :), this.newNode) + this.nearNodes(ind, 7);

                        if tmp_value < mini_cost
                            mini_cost = tmp_value;
                            new_parent_id = this.nearNodes(ind, 9);
                        end

                    end

                    ind = ind + 1;
                else %删去集合中有碰撞的附近点
                    this.nearNodes(ind, :) = [];
                    ind = ind - 1;
                    n = n - 1;

                    if ind == 0
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
                        tmp_value = this.calc_cost(this.newNode, this.nearNodes(i, :)) + this.newNode(7);

                        if tmp_value < this.nearNodes(i, 7)
                            % id parentid newparentid
                            this.replot_num = this.replot_num + 1;
                            this.replot (this.replot_num, :) = [this.nearNodes(i, 9) this.nearNodes(i, 8) this.newNode(9)];
                            this.nearNodes(i, 7) = tmp_value;
                            this.nearNodes(i, 8) = this.newNode(9);
                        end

                    end

                end

            end

        end

        function ind=rewire_v2(this, new_parent_id)
            %重布线
            %tmp_value(1,1)=find(this.nearNodes(:, 9) == new_parent_id);
            %this.nearNodes(tmp_value(1,1), :) = [];
            this.nearNodes(find(this.nearNodes(:, 9) == new_parent_id), 11) = inf;
            compare_table = zeros(length(this.nearNodes(:, 1)), 1);
            compare_table(:, 1) = this.nearNodes(:, 11) + this.newNode(7); %新节点作为附近点的父节点后的代价
            index = (compare_table(:, 1) - this.nearNodes(:, 7)) < 0; %替换后代价变小的点的下标
            ind = this.nearNodes(index, 9); %替换后代价变小的点的id
            [this.replot_num, ~] = size(ind(:, 1));
            this.rewire_num = this.rewire_num + this.replot_num;
            this.tree(ind, 10) = this.nearNodes(index, 11);
            this.tree(ind, 8) = this.newNode(9);

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

            % this.path(:,1:3)=this.path(:,1:3)*this.height_scale;
            % this.path(:,8:9)=this.path(:,8:9)*this.height_scale;

            % for ind = 1:6
            %     this.path(1:interp_num:path_num,ind)=interp1(1:path_num,this.path(1:path_num,ind ),1:interp_num:path_num,'spline');
            % end

        end

        function [new_path, num] = follow_ground(this, path_num)
            new_path = zeros(400, 10);
            ind = 1;

            for i = path_num:-1:2
                [target_h, delta_dist] = this.maps.checkPath_v2(this.path(i, 1:3), this.path(i - 1, 1:3));
                new_target_h = this.robot.just_follow(target_h(:, 3), delta_dist);
                target_h(:, 3) = new_target_h;
                [tmp_value, ~] = size(target_h);
                % new_path(ind, :) = this.path(i, :);
                % new_path(ind, 9) = this.maps.find_height(this.path(i, :));

                new_path(ind:ind + tmp_value - 2, 1:3) = target_h(1:tmp_value - 1, 1:3);
                new_path(ind:ind + tmp_value - 2, 9) = target_h(1:tmp_value - 1, 4); %地形高度

                ind = ind + tmp_value;
            end

            % new_path(ind, :) = this.path(1, :);
            % new_path(ind, 9) = this.maps.find_height(this.path(1, :));

            new_path(:, 8) = new_path(:, 3) - new_path(:, 9); %轨迹上的高度差
            new_path = new_path(new_path(:, 3) ~= 0, :);
            % for i = 1:ind
            %     % new_path(i, 9) = this.maps.Z(y, round(new_path(i, 2))); %轨迹对应的地形高度
            %     new_path(i, 8) = new_path(i, 3) - new_path(i, 9); %轨迹上的高度差
            % end

            % num = ind - 1;
            [num, ~] = size(new_path);
        end

        function output = to_normal_size(this, new_path, path_num)
            output = zeros(path_num, 3);
            output(:, 1:2) = new_path(:, 1:2) * this.map_scale;
            output(:, 3) = new_path(:, 3) * this.height_scale;

            output(:, 1) = output(:, 1) + min(this.maps.X);
            output(:, 2) = output(:, 2) + min(this.maps.Y);
            % output = this.path(:, 1:3);
        end

    end

    methods (Access = public)

        function [output, interp_num] = start_star(this, max_time)

            if nargin < 1
                max_time = 7;
            end

            this.tree = zeros(this.max_nodes, 10);
            % this.multi_state = zeros(this.max_nodes, 4);
            this.path = zeros(500, 10);
            this.nodenum = 1;
            this.informed = false;
            this.randnum = this.randnums(1, 1);
            this.search_num = 0;
            this.tooclose = 0;
            this.isgoal = 0;
            this.no_parent = 0;
            this.rewire_num = 0;

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
                this.newNode(1, 1:6) = this.robot.transfer_directly(sample, closestNode, this.maps.Z);

                if this.neighbors(neighbor_dist)
                    this.tooclose = this.tooclose + 1;
                    continue
                end

                new_id = this.choose_parent_v2();

                if norm(this.newNode(1:3) - this.goal(1:3)) > this.threshold_goal

                    if new_id > 0
                        this.insert_node(new_id);
                        this.rewire_v2(new_id);

                    else
                        this.no_parent = this.no_parent + 1;
                    end

                else %if flag == 2
                    this.isgoal = this.isgoal + 1;
                    % new_id
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
            %fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d', this.search_num, this.tooclose, this.isgoal, this.no_parent, this.rewire_num);
            % interp_num = this.interpolation(path_num);
            [new_path, interp_num] = this.follow_ground(path_num);

            %this.path = new_path;
            % this.path_sample=this.path;
            % output = this.maps.to_normal_size(this.path);
            output = this.to_normal_size(new_path, interp_num);

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
            s(3) = this.maps.Z(s(1), s(2)) + s(3) / this.height_scale;
            g(3) = this.maps.Z(g(1), g(2)) + g(3) / this.height_scale;

            this.start = s;
            this.goal = g;

        end

        function set_params(this, conf)
            this.height_cost_rate = conf.height_cost_rate;
            this.threshold_close = (conf.threshold_close / this.height_scale) ^ 2;
            this.threshold_goal = conf.threshold_goal / this.height_scale;
            this.max_nodes = conf.max_nodes;
            this.randnums = conf.randnum;
        end

        function this = rrt(conf)
            this.maps = map(conf.dem_data);
            conf.map_scale = 500; %
            this.map_scale = this.maps.X(1, 2) - this.maps.Y(1, 1);
            this.height_scale = conf.map_scale;
            this.maps.set_height_limit(conf.height_limit / this.height_scale);
            Height = this.maps.Z;
            this.start = [conf.start(1), conf.start(2), Height(conf.start(1), conf.start(2)) + conf.start(3) / this.height_scale, conf.start(4), conf.start(5), conf.start(6)];
            this.goal = [conf.goal(1), conf.goal(2), Height(conf.goal(1), conf.goal(2)) + conf.goal(3) / this.height_scale, conf.goal(4), conf.goal(5), conf.goal(6)];
            this.searchbBase = [1, 1, min(min(Height)), -pi, -pi, -pi];
            this.searchSize = [this.maps.X_num - 1, this.maps.Y_num - 1, max(max(Height)) - min(min(Height)), 2 * pi, 2 * pi, 2 * pi];
            %            this.replot = zeros(50, 3);
            this.robot = uav(conf);
            this.set_params(conf);

        end

    end

end
