classdef rrt < benchmark & tree

    properties (SetAccess = public)
        config_manger
        name = 'rrt'
        position = zeros(1, 3, 'single')
        cost_to_parent = zeros(1, 1, 'single')
        cost_to_root = zeros(1, 1, 'single')
        maps
        robot
        start_point = zeros(1, 6, 'single')
        goal = zeros(1, 6, 'single')
        threshold_close = 1
        threshold_goal = 1
        search_size = zeros(1, 6, 'single')
        search_base = zeros(1, 6, 'single')
        rand_num = 1
        height_cost_rate = 6
        neighbor_dist
        near_nodes = struct
        best_path
        rand_delete
        threshold_close_rate
        threshold_goal_rate
        neighbor_range_rate
        %informed
        informed = false
        long_axis = 0
        short_axis = 0
        ellipse_rotation = zeros(2, 2, 'single')
        ellipse_displace = zeros(2, 1, 'single')
        search_area = 0
        search_area_rate
        %output
        num_iter = 0
        num_close = 0
        num_goal = 0
        num_no_parent = 0
        num_rewire = 0
        num_neighbor = 0
        stamp = zeros(6, 1, 'single');
        path_id = []
        mini_path_len = inf
        %temporary
        compare_all = zeros(1, 2, 'single')
        compare_near = zeros(1, 1, 'single')
        is_delete = false;
        repeat_times = 1;
        use_informed = true;
        start
    end

    methods (Access = public)

        function sample = get_sample(this)

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
            this.compare_all = this.position(1:this.node_num, 1:2) - this.new_node.position(1, 1:2);
            this.compare_all(:, 1) = sum(this.compare_all(:, 1:2) .^ 2, 2);
            [tmp_value, ~] = min(this.compare_all(:, 1));

            if tmp_value < this.threshold_close
                this.num_close = this.num_close + 1;
                flag = true;
                return
            end

            flag = false;
            this.near_nodes = struct;
            this.near_nodes.id = find(this.compare_all(:, 1) < this.neighbor_dist);
            this.near_nodes.position = this.position(this.near_nodes.id, :);
            this.near_nodes.num = length(this.near_nodes.id);
            this.near_nodes.cost_to_root = this.cost_to_root(this.near_nodes.id, 1);
            this.near_nodes.cost_to_newNode = zeros(this.near_nodes.num, 1);
            this.compare_near = zeros(this.near_nodes.num, 1);
        end

        function [node, index] = get_closest(this, sample)
            this.compare_all = this.position(1:this.node_num, 1:2) - sample(1:2);
            [~, index] = min(sum(this.compare_all(:, 1:2) .^ 2, 2));
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
            % cost = sqrt(cost_dist ^ 2 + cost_hight ^ 2);
            cost = cost_dist + cost_hight;
        end

        function insert_node(this)
            insert_node@tree(this);
            this.position(this.new_node.id, :) = this.new_node.position;
            this.cost_to_parent(this.new_node.id, 1) = this.new_node.cost_to_parent;
            this.cost_to_root(this.new_node.id, 1) = this.new_node.cost_to_root;
        end

        function path = trace_back(this, id)
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
            this.compare_near(:, 1:2) = this.near_nodes.position(:, 1:2) - this.new_node.position(1, 1:2);
            this.compare_near(:, 1) = sqrt(sum(this.compare_near .^ 2, 2));
            f = false(this.near_nodes.num, 1);

            for i = 1:this.near_nodes.num
                [ground_h, f(i)] = this.maps.checkPath(this.near_nodes.position(i, :), this.new_node.position(1, :));
                this.compare_near(i, 2) = this.height_cost_rate * sum(abs(diff(ground_h)));
            end

            this.compare_near(:, 1) = sqrt(this.compare_near(:, 1) .^ 2 + this.compare_near(:, 2) .^ 2);
            this.compare_near(~f, 1) = inf;
            this.near_nodes.cost_to_newNode(:, 1) = this.compare_near(:, 1);
            % this.near_nodes.cost_to_root(:, 1) = this.cumcost(this.near_nodes.id(:, 1));
            this.compare_near = this.near_nodes.cost_to_newNode(:, 1) + this.near_nodes.cost_to_root(:, 1);
            [this.new_node.cost_to_root, ind] = min(this.compare_near(:, 1));
            this.new_node.cost_to_parent = this.near_nodes.cost_to_newNode(ind, 1);
            this.new_node.id_parent = this.near_nodes.id(ind, 1);
        end

        function choose_parent_v3(this)
            cost_dist = sqrt(sum((this.near_nodes.position(:, 1:2) - this.new_node.position(1, 1:2)) .^ 2, 2));
            cost_h = this.height_cost_rate * (this.near_nodes.position(:, 3) - this.new_node.position(1, 3));
            tmp_value = sqrt(cost_dist .^ 2 + cost_h .^ 2); %
            % this.near_nodes.cost_to_root(:, 1) = this.cumcost(this.near_nodes.id(:, 1));
            this.near_nodes.cost_to_newNode(:, 1) = tmp_value;
            this.compare_near = this.near_nodes.cost_to_newNode(:, 1) + this.near_nodes.cost_to_root(:, 1);
            [this.new_node.cost_to_root, ind] = min(this.compare_near(:, 1));
            this.new_node.cost_to_parent = this.near_nodes.cost_to_newNode(ind, 1);
            this.new_node.id_parent = this.near_nodes.id(ind, 1);
        end

        function id_rewire = rewire(this)

            if this.near_nodes.num == 1 %no need to rewire
                id_rewire = []; return
            end

            this.compare_near(:, 1) = this.near_nodes.cost_to_newNode(:, 1) + this.new_node.cost_to_root;
            index_rewire = find(this.compare_near(:, 1) < this.near_nodes.cost_to_root(:, 1));
            id_rewire = this.near_nodes.id(index_rewire, 1);
            this.num_rewire = this.num_rewire + length(id_rewire);
            this.cost_to_parent(id_rewire, 1) = this.near_nodes.cost_to_newNode(index_rewire, 1);
            % this.parent(id_rewire, 1) = this.new_node.id;
            this.change_parent(id_rewire, this.new_node.id);

            for i = 1:length(id_rewire)
                offspring = this.get_offspring(id_rewire(i));

                if ~isempty(offspring)
                    this.cost_to_root(offspring, 1) = this.cost_to_root(offspring, 1) + this.compare_near(index_rewire(i), 1) - this.near_nodes.cost_to_root(index_rewire(i), 1);
                end

            end

            this.cost_to_root(id_rewire, 1) = this.compare_near(index_rewire, 1);
        end

        function id_rewire = rewire2(this)

            if this.near_nodes.num == 1 %no need to rewire
                id_rewire = []; return
            end

            this.compare_near(:, 1) = this.near_nodes.cost_to_newNode(:, 1) + this.new_node.cost_to_root; %near new cost
            index_rewire = find(this.compare_near(:, 1) < this.near_nodes.cost_to_root(:, 1));

            if isempty(index_rewire)
                id_rewire = []; return
            end

            id_rewire = this.near_nodes.id(index_rewire, 1);
            delta_cost = this.compare_near(index_rewire, 1) - this.near_nodes.cost_to_root(index_rewire, 1);
            % this.parent(id_rewire, 1) = this.new_node.id;
            preorder_id = this.preorder(ismember(this.preorder, id_rewire), 1);
            flag = false;

            for i = length(preorder_id):-1:2
                offspring = this.get_offspring(preorder_id(i - 1));

                if ismember(preorder_id(i), offspring) && delta_cost(id_rewire == preorder_id(i - 1)) < delta_cost(id_rewire == preorder_id(i - 1))
                    index_rewire(id_rewire == preorder_id(i)) = [];
                    flag = true;
                end

            end

            if flag
                id_rewire = this.near_nodes.id(index_rewire, 1);
                delta_cost = this.compare_near(index_rewire, 1) - this.near_nodes.cost_to_root(index_rewire, 1);
            end

            this.change_parent(id_rewire, this.new_node.id);

            for i = 1:length(id_rewire)
                offspring = this.get_offspring(id_rewire(i));

                if ~isempty(offspring)
                    this.cost_to_root(offspring, 1) = this.cost_to_root(offspring, 1) + delta_cost(i); %this.compare_near(index_rewire(i), 1) - this.near_nodes.cost_to_root(index_rewire(i), 1);
                end

            end

            this.num_rewire = this.num_rewire + length(id_rewire);
            this.cost_to_parent(id_rewire, 1) = this.near_nodes.cost_to_newNode(index_rewire, 1);
            this.cost_to_root(id_rewire, 1) = this.compare_near(index_rewire, 1);
        end

        function prepare_informed(this, path_len)
            this.informed = true;
            dist = norm(this.start_point(1:3) - this.goal(1:3)) / 2;
            this.long_axis = path_len / 2;
            this.short_axis = sqrt(this.long_axis ^ 2 - dist ^ 2);
            ang = atan2(this.goal(2) - this.start_point(2), this.goal(1) - this.start_point(1));
            this.ellipse_rotation = [cos(ang) -sin(ang); sin(ang) cos(ang)];
            this.ellipse_displace = [dist * cos(ang); dist * sin(ang)] + [this.start_point(1); this.start_point(2)];
            this.search_area = pi * this.long_axis * this.short_axis;
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

        function delete_unuesd_node(this)
            id_remain = unique(this.parent(:, 1));
            id_remain = id_remain(id_remain > 0);
            id_remain = [id_remain; this.path_id];
            id_remain = sortrows(unique(id_remain));
            old_id = zeros(this.node_num, 1, 'uint32');
            old_id(:, 1) = 1:this.node_num;
            id_delete = old_id(~ismember(old_id, id_remain));
            id_delete = id_delete(rand(length(id_delete), 1) < this.rand_delete);
            id_remain = old_id(~ismember(old_id, id_delete));
            this.delete_node(id_remain, id_delete);
            this.position(id_delete, :) = [];
            this.cost_to_parent(id_delete) = [];
            this.cost_to_root(id_delete) = [];
            this.path_id = this.mapping(this.path_id);
        end

        function path = find_path(this)
            this.num_goal = this.num_goal + 1;
            this.path_id(this.num_goal, 1) = this.new_node.id;
            path = this.trace_back(this.new_node.id);
            path = [this.goal(1:3); path];
            tmp_value = path(2:end, 1:3) - path(1:end - 1, 1:3);
            path_len = sum(sqrt(sum(tmp_value .^ 2, 2)));

            if this.mini_path_len > path_len
                this.best_path = path;
                this.mini_path_len = path_len;

                if this.use_informed
                    this.prepare_informed(path_len);
                end

            end

        end

        function result = record_fun(this)
            [result.mini_cost, ~] = this.find_best_path();
            result.direct_step = this.robot.get_step();
        end

        function rewire_all(this)
            this.num_rewire = 0;
            this.compare_all = zeros(this.node_num, 1, 'single');

            for i = 1:this.node_num
                this.new_node.id = i;
                this.new_node.cost_to_root = this.cost_to_root(i);
                this.new_node.position = this.position(i, :);
                this.new_node.id_parent = this.parent(i);
                this.new_node.cost_to_parent = this.cost_to_parent(i);
                this.neighbors();

                this.compare_near = this.near_nodes.position(:, 1:2) - this.new_node.position(1, 1:2);
                this.compare_near(:, 1) = sqrt(sum(this.compare_near .^ 2, 2));
                f = false(this.near_nodes.num, 1);

                for j = 1:this.near_nodes.num
                    [ground_h, f(j)] = this.maps.checkPath(this.near_nodes.position(j, :), this.new_node.position(1, :));
                    this.compare_near(j, 2) = this.height_cost_rate * sum(abs(diff(ground_h)));
                end

                this.compare_near(:, 1) = sqrt(this.compare_near(:, 1) .^ 2 + this.compare_near(:, 2) .^ 2);
                this.compare_near(~f, 1) = inf;
                this.near_nodes.cost_to_newNode(:, 1) = this.compare_near(:, 1);

                this.rewire();
            end

            this.num_rewire

        end

        function update_step(this)

            if this.informed
                r = sqrt(this.search_area * this.search_area_rate / this.node_num / pi);
                this.robot.update_step(r / this.neighbor_range_rate);
                this.threshold_close = (this.robot.get_threshold(this.threshold_close_rate)) ^ 2;
                this.threshold_goal = this.robot.get_threshold(this.threshold_goal_rate);
                this.neighbor_dist = this.robot.get_threshold(this.neighbor_range_rate) ^ 2;
            end

        end

    end

    methods (Access = public)

        function start_rrt(this)
            this.mini_path_len = inf;
            t = tic;

            while this.record(toc(t), this.num_iter)
                this.num_iter = this.num_iter + 1;
                sample = this.get_sample();
                [closest_node, parent_id] = this.get_closest(sample);
                this.new_node.position(1, :) = this.robot.transfer(sample, closest_node);
                this.new_node.id_parent = parent_id;

                if this.new_node.id_parent > 0
                    this.insert_node();
                else
                    this.num_no_parent = this.num_no_parent + 1;
                end

                if norm(this.new_node.position(1:3) - this.goal(1:3)) < this.threshold_goal
                    this.find_path(mini_path_len);
                    break;
                end

            end

            toc(t)
            [~, path] = this.find_best_path();
            this.best_path = this.follow_ground(path);
        end

        function start_direct(this, t, start_dying_step)

            if nargin == 1
                t = tic;
                start_dying_step=0;
            end

            this.mini_path_len = inf;

            while this.record(toc(t), this.num_iter)
                this.num_iter = this.num_iter + 1;
                sample = this.get_sample();
                [closest_node, ~] = this.get_closest(sample);
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
                    this.find_path();
                end

                if this.is_delete && this.node_num > this.max_nodes
                    this.delete_unuesd_node();
                end

                if this.search_area_rate > 0
                    this.update_step();

                    if this.robot.get_step() < start_dying_step
                        return
                    end

                end

            end

            toc(t)
            [cost, path] = this.find_best_path();
            fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d\n邻居个数%d\n路径代价为%f\n', this.num_iter, this.num_close, this.num_goal, this.num_no_parent, this.num_rewire, this.num_neighbor, cost);
            this.best_path = this.follow_ground(path);
        end

        function start_kinetic(this)
            this.mini_path_len = inf;
            t = tic;

            while this.record(toc(t), this.num_iter)
                this.num_iter = this.num_iter + 1;
                sample = this.get_sample();
                [closest_node, ~] = this.get_closest(sample);
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
                    this.find_path();
                end

                if this.is_delete && this.node_num > this.max_nodes
                    this.delete_unuesd_node();
                end

                if this.search_area_rate > 0
                    this.update_step();
                end

            end

            toc(t)
            [cost, path] = this.find_best_path();
            fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d\n邻居个数%d\n路径代价为%f\n', this.num_iter, this.num_close, this.num_goal, this.num_no_parent, this.num_rewire, this.num_neighbor, cost);
            this.best_path = this.follow_ground(path);
        end

        function get_new_config(this, config_dir)
            get_new_config@benchmark(this)
            this.config_manger = configs.get_config([this.name, '_', config_dir]);
            this.maps.get_new_config(config_dir);
            this.robot.get_new_config(config_dir);
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
            this.threshold_close = (this.robot.get_threshold(conf.threshold_close)) ^ 2;
            this.threshold_goal = (this.robot.get_threshold(conf.threshold_goal));
            this.max_nodes = conf.delete_node.max_nodes;
            this.is_delete = conf.delete_node.is_delete;
            this.rand_delete = conf.delete_node.rand_delete;
            init@tree(this);
            this.rand_num = conf.rand_num;
            this.neighbor_dist = this.robot.get_threshold(conf.neighbor_range) ^ 2;
            this.position = zeros(this.max_nodes, this.robot.dimension, 'single');
            this.cost_to_parent = zeros(this.max_nodes, 1, 'single');
            this.cost_to_root = zeros(this.max_nodes, 1, 'single');
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
            this.start_benchmark();
            this.threshold_close_rate = conf.threshold_close;
            this.threshold_goal_rate = conf.threshold_goal;
            this.neighbor_range_rate = conf.neighbor_range;
            this.search_area_rate = conf.search_area_rate;
            this.use_informed = conf.use_informed;

            switch conf.start
                case 'start_direct'
                    this.start = @this.start_direct;
                case 'start_classify_rrt'
                    this.start = @this.start_classify_rrt;
            end

            if isfield(conf, 'repeat_times')
                this.repeat_times = conf.repeat_times;
            else
                this.repeat_times = 1;
            end

        end

        function output = get_repeat_times(this)
            output = this.repeat_times;
        end

        function save_all(this, annotation)
            conf = this.config_manger.load(this.rand_id);

            if conf.is_save

                if nargin == 1
                    path = save_all@benchmark(this);
                elseif nargin == 2
                    path = save_all@benchmark(this, annotation);
                end

                this.maps.config_manger.save(path);
                this.robot.config_manger.save(path);
            end

        end

        function this = rrt()
            this = this@benchmark();
            this.config_manger = configs.get_config(this.name);
            this.maps = map.get_instance();
            this.robot = uav();
        end

    end

end
