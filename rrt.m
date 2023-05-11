classdef rrt < handle

    properties (SetAccess = public)
        tree = zeros(1, 3)
        cost_to_parent = zeros(1, 1)
        parent = zeros(1, 1)
        max_nodes = 100
        maps
        robot
        start = zeros(1, 6)
        goal = zeros(1, 6)
        threshold_close = 1
        threshold_goal = 1
        search_size = zeros(1, 6)
        search_base = zeros(1, 6)
        rand_num = 1
        rand_nums = zeros(1, 2)
        % multi_state
        height_cost_rate = 6
        neighbor_dist

        near_nodes = struct
        new_node = struct
        node_num = 10
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
        path_id = []
    end

    methods (Access = public)

        function sample = get_sample(this)
            %采样

            if this.informed
                y = 0;
                sample = [0 inf 0]';

                while abs(sample(2)) > y
                    sample(1:2) = [rand * 2 * this.long_axis; rand * 2 * this.short_axis] - [this.long_axis; this.short_axis];
                    y = this.short_axis * sqrt(1 - (sample(1) / this.long_axis) ^ 2);
                end

                sample(3) = rand * this.search_size(3) + this.search_base(3);
                sample(1:2) = this.ellipse_rotation * sample(1:2) + this.ellipse_displace;
                sample = sample';
            else

                if rand < this.rand_num

                    sample = rand(1, 3) .* this.search_size(1, 1:3) + this.search_base(1, 1:3);
                else
                    sample = this.goal(1, 1:3);
                end

            end

        end

        function flag = neighbors(this)
            %找附近的点
            flag = 0;
            compare_table = zeros(this.node_num - 1, 2);
            compare_table(:, 1:2) = this.tree(1:this.node_num - 1, 1:2) - this.new_node.position(1, 1:2);
            compare_table(:, 1) = sum(compare_table(:, 1:2) .^ 2, 2);
            [tmp_value, ~] = min(compare_table(:, 1)); %最近的距离

            if tmp_value < this.threshold_close %太近
                flag = 1;
            end

            this.near_nodes = struct;
            this.near_nodes.id = find(compare_table(:, 1) < this.neighbor_dist);
            this.near_nodes.position = this.tree(this.near_nodes.id, :);
            this.near_nodes.parent = this.parent(this.near_nodes.id, :);
            this.near_nodes.num = length(this.near_nodes.id);
            this.near_nodes.cost = zeros(this.near_nodes.num, 1);
            this.near_nodes.cost_to_newNode = zeros(this.near_nodes.num, 1);
        end

        function [node, index] = get_closest(this, sample)
            %找最近点
            compare_table = this.tree(1:this.node_num - 1, 1:2) - sample(1:2);
            [~, index] = min(sum(compare_table(:, 1:2) .^ 2, 2));
            node = this.tree(index, :);
        end

        function cost = calc_cost(~, from_node, dest_node)
            %计算相邻两点的代价
            cost_dist = norm(from_node(1:2) - dest_node(1:2));
            cost_hight = 6 * norm(from_node(3) - dest_node(3));
            cost_angle = norm(from_node(4:6) - dest_node(4:6));
            %consumption = this.robot.calc_consumption();
            cost = norm([cost_dist, cost_angle, cost_hight]);
        end

        function ids = get_ancestor(this, prev)
            ids = zeros(1, 1);
            ids(1, 1) = prev;
            ind = 2;
            prev = this.parent(prev, 1);

            while prev > 0
                ids(ind, 1) = prev;
                ind = ind + 1;
                prev = this.parent(prev, 1);
            end

        end

        function output = cumcost(this, prev_list)
            output = zeros(length(prev_list), 1);

            for i = 1:length(prev_list)
                ids = this.get_ancestor(prev_list(i));
                output(i) = sum(this.cost_to_parent(ids, 1));
            end

        end

        function cost = calc_cost_v2(this, from_node, dest_node, new_target_h)
            cost_dist = norm(from_node(1:2) - dest_node(1:2));
            cost_hight = this.height_cost_rate * sum(abs(diff(new_target_h)));
            cost = norm([cost_dist, cost_hight]);

        end

        function insert_node(this)
            %插入节点
            this.tree(this.node_num, :) = this.new_node.position;
            this.parent(this.node_num, 1) = this.new_node.id_parent;
            this.cost_to_parent(this.node_num, 1) = this.new_node.cost_to_parent;
            this.new_node.id = this.node_num;
            this.node_num = this.node_num + 1;
        end

        function path = trace_back(this, id)
            %回溯轨迹
            ids = this.get_ancestor(id);
            path = this.tree(ids, 1:3);
        end

        function [min_path_cost, path] = find_best_path(this)
            path_costs = this.cumcost(this.path_id);
            [min_path_cost, best_path_id] = min(path_costs);
            path = this.trace_back(this.path_id(best_path_id));
        end

        function choose_parent_v2(this)
            % this.near_nodes.cost(:, 1) = this.cumcost(this.near_nodes.id(:, 1));

            for i = 1:this.near_nodes.num
                [target_h, delta_dist, flag, c] = this.maps.checkPath_v2(this.near_nodes.position(i, :), this.new_node.position(1, :));

                % if f
                %     [new_target_h, ~] = this.robot.follow(this.near_nodes.position(i, :), this.new_node.position(1, :), target_h(:, 3), delta_dist);
                %     tmp_value = this.calc_cost_v2(this.near_nodes.position(i, :), this.new_node.position(1, :), new_target_h); % 新节点与附近的相邻转移代价 + this.near_nodes(i, 7);
                %     tmp_value = norm([tmp_value, c]);
                % else
                %     tmp_value = inf;
                % end

                if flag
                    [new_target_h, flag2] = this.robot.follow(this.near_nodes.position(i, :), this.new_node.position(1, :), target_h(:, 3), delta_dist);

                    if flag2
                        tmp_value = this.calc_cost_v2(this.near_nodes.position(i, :), this.new_node.position(1, :), new_target_h); % 新节点与附近的相邻转移代价 + this.near_nodes(i, 7);
                        this.near_nodes.cost(i, 1) = this.cumcost(this.near_nodes.id(i, 1));
                        tmp_value = norm([tmp_value, c]);
                    else
                        tmp_value = inf;
                    end

                else
                    tmp_value = inf;
                end

                this.near_nodes.cost_to_newNode(i, 1) = tmp_value;
            end

            % compare_table = zeros(this.near_nodes.num, 1);
            compare_table = this.near_nodes.cost_to_newNode(:, 1) + this.near_nodes.cost(:, 1); %以附近点作新节点父亲后的代价
            [this.new_node.cost, ind] = min(compare_table(:, 1));
            this.new_node.cost_to_parent = this.near_nodes.cost_to_newNode(ind, 1);
            this.new_node.id_parent = this.near_nodes.id(ind, 1);

        end

        function output = calc_cost_v3(this)
            cost_dist = sqrt(sum((this.near_nodes.position(:, 1:2) - this.new_node.position(1, 1:2)) .^ 2, 2));
            cost_h = this.height_cost_rate * (this.near_nodes.position(:, 3) - this.new_node.position(1, 3));
            output = sqrt(cost_dist .^ 2 + cost_h .^ 2);
        end

        function choose_parent_v3(this)

            tmp_value = this.calc_cost_v3(); % 新节点与附近的相邻转移代价 + this.near_nodes(i, 7);
            this.near_nodes.cost(:, 1) = this.cumcost(this.near_nodes.id(:, 1));
            this.near_nodes.cost_to_newNode(:, 1) = tmp_value;

            % compare_table = zeros(this.near_nodes.num, 1);
            compare_table = this.near_nodes.cost_to_newNode(:, 1) + this.near_nodes.cost(:, 1); %以附近点作新节点父亲后的代价
            [this.new_node.cost, ind] = min(compare_table(:, 1));
            this.new_node.cost_to_parent = this.near_nodes.cost_to_newNode(ind, 1);
            this.new_node.id_parent = this.near_nodes.id(ind, 1);

        end

        function ind = rewire_v2(this)
            %重布线
            this.near_nodes.cost_to_newNode(find(this.near_nodes.id(:, 1) == this.new_node.id_parent), 1) = inf;
            compare_table = zeros(this.near_nodes.num, 1);
            compare_table(:, 1) = this.near_nodes.cost_to_newNode(:, 1) + this.new_node.cost; %新节点作为附近点的父节点后的代价
            index = (compare_table(:, 1) - this.near_nodes.cost(:, 1)) < 0; %替换后代价变小的点的下标
            ind = this.near_nodes.id(index, 1); %替换后代价变小的点的id
            [this.replot_num, ~] = size(ind(:, 1));
            this.rewire_num = this.rewire_num + this.replot_num;
            this.cost_to_parent(ind, 1) = this.near_nodes.cost_to_newNode(index, 1);
            this.parent(ind, 1) = this.new_node.id;

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

        function new_path = follow_ground(this, path)
            new_path = zeros(400, 10);
            ind = 1;
            path_num = length(path(:, 1));

            for i = path_num:-1:2
                [target_h, delta_dist, ~, ~] = this.maps.checkPath_v2(path(i, 1:3), path(i - 1, 1:3));
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
            %     new_path(i, 9) = this.maps.Z(y, round(new_path(i, 2))); %轨迹对应的地形高度
            %     new_path(i, 8) = new_path(i, 3) - new_path(i, 9); %轨迹上的高度差
            % end

            % num = ind - 1;
        end

        function output = to_normal_size(this, new_path)
            output = zeros(path_num, 3);
            output(:, 1:2) = new_path(:, 1:2) * this.map_scale;
            output(:, 3) = new_path(:, 3) * this.height_scale;
            output(:, 1) = output(:, 1) + min(this.maps.X);
            output(:, 2) = output(:, 2) + min(this.maps.Y);
            % output = this.path(:, 1:3);
        end

        function ind_parent = delete_unuesd_node(this)
            ind_parent = unique(this.parent(:, 1));
            ind_parent(ind_parent < 0) = [];
            ind_parent = [ind_parent; this.path_id];
            ind_parent = sortrows(unique(ind_parent));
            this.tree = this.tree(ind_parent, :);
            this.node_num = size(this.tree, 1) + 1;
            id_map = containers.Map(ind_parent, 1:(this.node_num - 1));

            for i = 1:length(this.path_id)
                this.path_id(i) = id_map(this.path_id(i));
            end

            for i = 2:this.node_num - 1
                this.parent(i, 1) = id_map(this.parent(i, 1));
            end

        end

    end

    methods (Access = public)

        function output = start_star(this, max_time)

            if nargin < 1
                max_time = 7;
            end

            this.tree = zeros(this.max_nodes, 3);
            this.parent = zeros(this.max_nodes, 1);
            this.node_num = 1;
            this.informed = false;
            this.rand_num = this.rand_nums(1, 1);
            this.search_num = 0;
            this.tooclose = 0;
            this.isgoal = 0;
            this.no_parent = 0;
            this.rewire_num = 0;

            this.path_id = [];
            this.new_node.position = this.start(1, 1:3);
            this.new_node.id_parent = -1;
            this.new_node.cost_to_parent = 0;
            this.insert_node();
            %(2 * this.robot.v * this.robot.deltaT) ^ 2;
            tic
            output = zeros(1, 1);
            ind = 1;
            last = 0;
            mini_path_len = inf;

            while toc <= max_time
                this.search_num = this.search_num + 1;
                sample = this.get_sample();
                [closest_node, ~] = this.get_closest(sample);
                this.new_node.position(1, :) = this.robot.transfer_directly(sample, closest_node);

                if this.neighbors()
                    this.tooclose = this.tooclose + 1;
                    continue
                end

                this.choose_parent_v2();

                if norm(this.new_node.position(1:3) - this.goal(1:3)) > this.threshold_goal

                    if this.new_node.id_parent > 0
                        this.insert_node();
                        this.rewire_v2();

                    else
                        this.no_parent = this.no_parent + 1;
                    end

                else
                    this.isgoal = this.isgoal + 1;
                    this.insert_node();
                    this.path_id(this.isgoal, 1) = this.new_node.id;
                    this.rand_num = this.rand_nums(1, 2); %搜索点不取goal
                    path = this.trace_back(this.new_node.id);
                    path = [this.goal(1:3); path; this.start(1:3)];
                    tmp_value = path(2:end, 1:3) - path(1:end - 1, 1:3);
                    path_len = sum(sqrt(sum(tmp_value .^ 2, 2)));

                    if mini_path_len > path_len
                        mini_path_len = path_len;
                        this.prepare_informed(path_len);
                    end

                end

                % if toc - last > 0.5
                %     last = toc;
                %     len = this.find_min_cost();
                %     output(ind) = len;
                %     ind = ind + 1;
                % end
                % if this.node_num > this.max_nodes
                %     this.delete_unuesd_node();
                % end

            end

            [c, path] = this.find_best_path();
            fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d', this.search_num, this.tooclose, this.isgoal, this.no_parent, this.rewire_num);
            fprintf('\n路径代价为%f', c);
            % interp_num = this.interpolation(path_num);
            %output = this.follow_ground(path);
            output =path;
            % output = new_path(:, 1:3);
        end

        function set_start_end(this, s, g)
            s(3) = this.maps.Z(s(1), s(2)) + s(3) / this.height_scale;
            g(3) = this.maps.Z(g(1), g(2)) + g(3) / this.height_scale;
            this.start = s;
            this.goal = g;
        end

        function set_params(this, conf)
            this.start = [conf.start(1), conf.start(2), this.maps.Z(conf.start(1), conf.start(2)) + conf.start(3) / this.height_scale, conf.start(4), conf.start(5), conf.start(6)];
            this.goal = [conf.goal(1), conf.goal(2), this.maps.Z(conf.goal(1), conf.goal(2)) + conf.goal(3) / this.height_scale, conf.goal(4), conf.goal(5), conf.goal(6)];
            this.height_cost_rate = conf.height_cost_rate;
            this.threshold_close = (conf.threshold_close / this.height_scale) ^ 2;
            this.threshold_goal = conf.threshold_goal / this.height_scale;
            this.max_nodes = conf.max_nodes;
            this.rand_nums = conf.rand_num;
            this.neighbor_dist = (2 * conf.direct_step) ^ 2;
            this.robot.set_params(conf)
        end

        function save(this)
            col = {'tree', 'parent', 'cost'};
            result_table = table(this.tree, this.parent, this.cost_to_parent, 'VariableNames', col);
            writetable(result_table, 'test.csv');

        end

        function this = rrt(conf)
            this.maps = map.get_instance(conf.dem_data);
            this.map_scale = conf.map_scale; %this.maps.X(1, 2) - this.maps.X(1, 1);
            this.height_scale = conf.map_scale;
            this.maps.set_height_limit(conf.height_limit / this.height_scale);
            this.search_base = [1, 1, min(min(this.maps.Z)), -pi, -pi, -pi];
            this.search_size = [this.maps.X_num - 1, this.maps.Y_num - 1, max(max(this.maps.Z)) - min(min(this.maps.Z)), 2 * pi, 2 * pi, 2 * pi];
            this.robot = uav(conf);
            this.set_params(conf);

        end

    end

end
