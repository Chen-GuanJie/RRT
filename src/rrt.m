classdef rrt < benchmark & tree

    properties (SetAccess = public)
        config_manger
        name = 'rrt'
        position = zeros(1, 3)
        cost_to_parent = zeros(1, 1)
        cost_to_root = zeros(1, 1)
        maps
        robot
        start_point = zeros(1, 6)
        goal = zeros(1, 6)
        threshold_close = 1
        threshold_goal = 1
        search_size = zeros(1, 6)
        search_base = zeros(1, 6)
        rand_num = 1
        height_cost_rate = 6
        neighbor_dist
        near_nodes = struct
        best_path
        %informed
        informed = false
        long_axis = 0
        short_axis = 0
        ellipse_rotation = zeros(2, 2)
        ellipse_displace = zeros(2, 1)
        %output
        num_iter = 0
        num_close = 0
        num_goal = 0
        num_no_parent = 0
        num_rewire = 0
        num_neighbor = 0;
        path_id = []
    end

    methods (Access = public)

        function sample = get_sample(this)
            %采样

            if this.informed
                y = 0;
                sample = [0; inf];

                while abs(sample(2)) > y
                    sample(1:2) = [rand * 2 * this.long_axis; rand * 2 * this.short_axis] - [this.long_axis; this.short_axis];
                    y = this.short_axis * sqrt(1 - (sample(1) / this.long_axis) ^ 2);
                end

                % sample(3) = rand * this.search_size(3) + this.search_base(3);
                sample(1:2) = this.ellipse_rotation * sample(1:2) + this.ellipse_displace;
                sample = sample';
            else

                if rand < this.rand_num

                    sample = rand(1, 2) .* this.search_size(1, 1:2) + this.search_base(1, 1:2);
                else
                    sample = this.goal(1, 1:2);
                end

            end

        end

        function flag = neighbors(this)
            %找附近的点
            flag = 0;
            compare_table = zeros(this.node_num - 1, 2);
            compare_table(:, 1:2) = this.position(1:this.node_num - 1, 1:2) - this.new_node.position(1, 1:2);
            compare_table(:, 1) = sum(compare_table(:, 1:2) .^ 2, 2);
            [tmp_value, ~] = min(compare_table(:, 1)); %最近的距离

            if tmp_value < this.threshold_close %太近
                this.num_close = this.num_close + 1;
                flag = 1;
            end

            this.near_nodes = struct;
            this.near_nodes.id = find(compare_table(:, 1) < this.neighbor_dist);
            this.near_nodes.position = this.position(this.near_nodes.id, :);
            this.near_nodes.num = length(this.near_nodes.id);
            this.near_nodes.cost_to_root = this.cost_to_root(this.near_nodes.id, 1);
            this.near_nodes.cost_to_newNode = zeros(this.near_nodes.num, 1);
        end

        function node = get_closest(this, sample)
            %找最近点
            compare_table = this.position(1:this.node_num - 1, 1:2) - sample(1:2);
            [~, index] = min(sum(compare_table(:, 1:2) .^ 2, 2));
            node = this.position(index, :);
        end

        function output = cumcost(this, prev_list)
            output = zeros(length(prev_list), 1);

            for i = 1:length(prev_list)
                ids = this.get_ancestor(prev_list(i));
                output(i, 1) = sum(this.cost_to_parent(ids, 1));
            end

        end

        function cost = calc_cost(this, from_node, dest_node, new_target_h)
            cost_dist = norm(from_node(1:2) - dest_node(1:2));
            cost_hight = this.height_cost_rate * sum(abs(diff(new_target_h)));
            cost = norm([cost_dist, cost_hight]);
        end

        function insert_node(this)
            %插入节点
            insert_node@tree(this);
            this.position(this.new_node.id, :) = this.new_node.position;
            this.cost_to_parent(this.new_node.id, 1) = this.new_node.cost_to_parent;
            this.cost_to_root(this.new_node.id, 1) = this.new_node.cost_to_root;
        end

        function path = trace_back(this, id)
            %回溯轨迹
            ids = this.get_ancestor(id);
            path = this.position(ids, 1:3);
        end

        function [min_path_cost, path] = find_best_path(this)

            if ~isempty(this.path_id)
                path_costs = this.cumcost(this.path_id);
                [min_path_cost, best_path_id] = min(path_costs);
                path = this.trace_back(this.path_id(best_path_id));
                path = [this.goal(1:3); path];
            else
                min_path_cost = inf;
                path = [];
            end

        end

        function choose_parent(this)

            for i = 1:this.near_nodes.num
                [ground_h, f] = this.maps.checkPath(this.near_nodes.position(i, :), this.new_node.position(1, :));

                if f
                    tmp_value = this.calc_cost(this.near_nodes.position(i, :), this.new_node.position(1, :), ground_h); % 新节点与附近的相邻转移代价 + this.near_nodes(i, 7);
                else
                    tmp_value = inf;
                end

                this.near_nodes.cost_to_newNode(i, 1) = tmp_value;
            end

            % this.near_nodes.cost_to_root(:, 1) = this.cumcost(this.near_nodes.id(:, 1));
            % compare_table = zeros(this.near_nodes.num, 1);
            compare_table = this.near_nodes.cost_to_newNode(:, 1) + this.near_nodes.cost_to_root(:, 1); %以附近点作新节点父亲后的代价
            [this.new_node.cost_to_root, ind] = min(compare_table(:, 1));
            this.new_node.cost_to_parent = this.near_nodes.cost_to_newNode(ind, 1);
            this.new_node.id_parent = this.near_nodes.id(ind, 1);
        end

        function choose_parent_v3(this)
            cost_dist = sqrt(sum((this.near_nodes.position(:, 1:2) - this.new_node.position(1, 1:2)) .^ 2, 2));
            cost_h = this.height_cost_rate * (this.near_nodes.position(:, 3) - this.new_node.position(1, 3));
            tmp_value = sqrt(cost_dist .^ 2 + cost_h .^ 2); % 新节点与附近的相邻转移代价 + this.near_nodes(i, 7);
            % this.near_nodes.cost_to_root(:, 1) = this.cumcost(this.near_nodes.id(:, 1));
            this.near_nodes.cost_to_newNode(:, 1) = tmp_value;
            % compare_table = zeros(this.near_nodes.num, 1);
            compare_table = this.near_nodes.cost_to_newNode(:, 1) + this.near_nodes.cost_to_root(:, 1); %以附近点作新节点父亲后的代价
            [this.new_node.cost_to_root, ind] = min(compare_table(:, 1));
            this.new_node.cost_to_parent = this.near_nodes.cost_to_newNode(ind, 1);
            this.new_node.id_parent = this.near_nodes.id(ind, 1);
        end

        function id_rewire = rewire(this)
            %重布线
            this.near_nodes.cost_to_newNode(find(this.near_nodes.id(:, 1) == this.new_node.id_parent), 1) = inf;
            compare_table = zeros(this.near_nodes.num, 1);
            compare_table(:, 1) = this.near_nodes.cost_to_newNode(:, 1) + this.new_node.cost_to_root; %新节点作为附近点的父节点后的代价
            index_rewire = find(compare_table(:, 1) < this.near_nodes.cost_to_root(:, 1)); %替换后代价变小的点的下标
            id_rewire = this.near_nodes.id(index_rewire, 1); %替换后代价变小的点的id
            this.num_rewire = this.num_rewire + length(id_rewire);
            this.cost_to_parent(id_rewire, 1) = this.near_nodes.cost_to_newNode(index_rewire, 1);

            for i = 1:length(id_rewire)
                this.change_parent(id_rewire(i), this.new_node.id);
                offspring = this.get_offspring(id_rewire(i), []);

                if ~isempty(offspring)
                    this.cost_to_root(offspring, 1) = this.cost_to_root(offspring, 1) + compare_table(index_rewire(i), 1) - this.near_nodes.cost_to_root(index_rewire(i), 1);
                end

            end

            this.cost_to_root(id_rewire, 1) = compare_table(index_rewire, 1);
            this.parent(id_rewire, 1) = this.new_node.id;
        end

        function prepare_informed(this, path_len)
            this.informed = true;
            dist = norm(this.start_point(1:3) - this.goal(1:3)) / 2;
            this.long_axis = path_len / 2;
            this.short_axis = sqrt(this.long_axis ^ 2 - dist ^ 2);
            ang = atan2(this.goal(2) - this.start_point(2), this.goal(1) - this.start_point(1));
            this.ellipse_rotation = [cos(ang) -sin(ang); sin(ang) cos(ang)];
            this.ellipse_displace = [dist * cos(ang); dist * sin(ang)] + [this.start_point(1); this.start_point(2)];
        end

        function new_path = follow_ground(this, path)

            if nargin == 1
                path = this.best_path;
            end

            new_path = zeros(1, 5);
            ind = 1;
            path_num = length(path(:, 1));

            for i = path_num:-1:2
                [ground_h, ~] = this.maps.checkPath(path(i, 1:2), path(i - 1, 1:2));
                target_h = this.robot.just_follow(ground_h, norm(path(i, 1:2) - path(i - 1, 1:2)));
                len = length(target_h);
                ind_x = path(i, 1):(path(i - 1, 1) - path(i, 1)) / (length(ground_h) - 1):path(i - 1, 1);
                ind_y = path(i, 2):(path(i - 1, 2) - path(i, 2)) / (length(ground_h) - 1):path(i - 1, 2);
                tmp = [ind_x; ind_y; target_h; ground_h]';
                new_path(ind:ind + len - 2, 1:4) = tmp(1:end - 1, 1:4);
                ind = ind + len - 1;
            end

            new_path(:, 3) = new_path(:, 3) +this.maps.height_limit;
            new_path(:, 5) = new_path(:, 3) - new_path(:, 4);
        end

        function ind_parent = delete_unuesd_node(this)
            ind_parent = unique(this.parent(:, 1));
            ind_parent(ind_parent < 0) = [];
            ind_parent = [ind_parent; this.path_id];
            ind_parent = sortrows(unique(ind_parent));
            this.position = this.position(ind_parent, :);
            this.node_num = size(this.position, 1) + 1;
            id_map = containers.Map(ind_parent, 1:(this.node_num - 1));

            for i = 1:length(this.path_id)
                this.path_id(i) = id_map(this.path_id(i));
            end

            for i = 2:this.node_num - 1
                this.parent(i, 1) = id_map(this.parent(i, 1));
            end

        end

        function [mini_path_len, path] = find_path(this, mini_path_len)
            this.num_goal = this.num_goal + 1;
            this.path_id(this.num_goal, 1) = this.new_node.id;
            path = this.trace_back(this.new_node.id);
            path = [this.goal(1:3); path];
            tmp_value = path(2:end, 1:3) - path(1:end - 1, 1:3);
            path_len = sum(sqrt(sum(tmp_value .^ 2, 2)));

            if mini_path_len > path_len
                mini_path_len = path_len;
                this.prepare_informed(path_len);
            end

        end

        function result = record_fun(this)
            [result.mini_cost, ~] = this.find_best_path();
        end

    end

    methods (Access = public)

        function start(this)
            mini_path_len = inf;
            t = tic;

            while this.record(toc(t), this.num_iter)
                this.num_iter = this.num_iter + 1;
                sample = this.get_sample();
                closest_node = this.get_closest(sample);
                this.new_node.position(1, :) = this.robot.transfer(sample, closest_node);

                if this.neighbors()
                    continue
                end

                this.num_neighbor = this.num_neighbor + this.near_nodes.num;
                this.choose_parent();

                if this.new_node.id_parent > 0
                    this.insert_node();
                    this.rewire();

                else
                    this.num_no_parent = this.num_no_parent + 1;
                end

                if norm(this.new_node.position(1:3) - this.goal(1:3)) < this.threshold_goal
                    [mini_path_len, ~] = this.find_path(mini_path_len);
                end

            end

            toc(t)
            [cost, path] = this.find_best_path();
            fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d\n邻居个数%d\n路径代价为%f\n', this.num_iter, this.num_close, this.num_goal, this.num_no_parent, this.num_rewire, this.num_neighbor, cost);
            this.best_path = this.follow_ground(path);
        end

        function times = statistic_time(this)
            mini_path_len = inf;
            times = zeros(1000, 4);
            ind_times = 1;
            stamp = zeros(1, 5);
            t = tic;

            while this.record(toc(t), this.num_iter)
                stamp(1, 1) = toc(t);
                this.num_iter = this.num_iter + 1;
                sample = this.get_sample();
                closest_node = this.get_closest(sample);
                this.new_node.position(1, :) = this.robot.transfer(sample, closest_node);
                stamp(1, 2) = toc(t); % 3.8569e-05

                if this.neighbors()
                    continue
                end

                stamp(1, 3) = toc(t); % 6.2915e-05
                this.num_neighbor = this.num_neighbor + this.near_nodes.num;
                this.choose_parent();
                stamp(1, 4) = toc(t); % 0.0012   total: 6.1588/7

                if this.new_node.id_parent > 0
                    this.insert_node();
                    this.rewire();
                    stamp(1, 5) = toc(t); %3.0134e-05
                    times(ind_times, :) = diff(stamp(1, :));
                    ind_times = ind_times + 1;
                else
                    this.num_no_parent = this.num_no_parent + 1;
                end

                if norm(this.new_node.position(1:3) - this.goal(1:3)) < this.threshold_goal
                    [mini_path_len, ~] = this.find_path(mini_path_len);
                end

            end

            toc(t)
            [cost, path] = this.find_best_path();
            fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d\n邻居个数%d\n路径代价为%f\n', this.num_iter, this.num_close, this.num_goal, this.num_no_parent, this.num_rewire, this.num_neighbor, cost);
            this.best_path = this.follow_ground(path);
        end

        function init(this)
            this.maps.init();
            this.robot.init();
            conf = this.config_manger.load(this.rand_id);
            this.start_point = this.maps.start_point;
            this.goal = this.maps.goal;
            this.search_base = [1, 1, min(min(this.maps.Z)), -pi, -pi, -pi];
            this.search_size = [this.maps.X_num - 1, this.maps.Y_num - 1, max(max(this.maps.Z)) - min(min(this.maps.Z)), 2 * pi, 2 * pi, 2 * pi];
            this.height_cost_rate = conf.height_cost_rate;
            this.threshold_close = (conf.threshold_close / this.maps.map_scale) ^ 2;
            this.threshold_goal = conf.threshold_goal / this.maps.map_scale;
            this.max_nodes = conf.max_nodes;
            init@tree(this);
            this.rand_num = conf.rand_num;
            this.neighbor_dist = this.robot.get_neighbor_dist(conf.neighbor_range);
            this.position = zeros(this.max_nodes, this.robot.dimension);
            this.cost_to_parent = zeros(this.max_nodes, 1);
            this.cost_to_root = zeros(this.max_nodes, 1);
            this.informed = false;
            this.num_iter = 0;
            this.num_close = 0;
            this.num_goal = 0;
            this.num_no_parent = 0;
            this.num_rewire = 0;
            this.num_neighbor = 0;
            this.path_id = [];
            this.new_node.cost_to_parent = 0;
            this.new_node.cost_to_root = 0;
            this.new_node.position = this.start_point(1, 1:this.robot.dimension);
            this.insert_node();
            this.start_benchmark(conf.benchmark);
        end

        function save_all(this)
            save_all@benchmark(this);
        end

        function this = rrt()
            this.config_manger = configs.get_config(this.name);
            this.maps = map.get_instance();
            this.robot = uav();
        end

    end

end
