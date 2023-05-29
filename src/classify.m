classdef classify < rrt

    properties (SetAccess = private)
        name_classify = 'classify'
        config_manger_classify
        influence = zeros(1, 1, 'single')
        on_path = zeros(1, 2, 'uint32') %id weight
        dying = zeros(1, 2, 'uint32') %id life
        dead = zeros(1, 1, 'uint32') %id
        alive = zeros(1, 1, 'uint32') %id
        T_delta = 100
        T_max = 500
        dying_speed = 1
        alpha = 20
        mini_dying_num = 5000
        start_dying_step = 4
        compare_alive
    end

    methods (Access = private)

        function calc_influence(this, changed_ind)
            w = single(this.on_path(changed_ind, 2));
            this.influence(changed_ind, 1) = this.alpha * sqrt(-log(0.1 ./ (w)) ./ (w .^ 2));
        end

        function tick(this)
            this.dying(:, 2) = this.dying(:, 2) - this.dying_speed;
            dead_ind = this.dying(:, 2) < 1;
            dead_id = this.dying(dead_ind, 1);

            if isempty(dead_id)
                return
            end

            this.alive(ismember(this.alive, dead_id), :) = [];
            this.dying(dead_ind, :) = [];
            this.dead = [this.dead; dead_id];
        end

        function weight_changed(this, changed_id, delta)

            if delta > 0
                in_path = ismember(changed_id, this.on_path(:, 1));
                add_path_id = changed_id(~in_path);
                ind = length(this.on_path) + (1:length(add_path_id));
                this.on_path(ind, 1) = add_path_id;
                this.on_path(ind, 2) = 0;
            end

            changed_ind = ismember(this.on_path(:, 1), changed_id);
            this.on_path(changed_ind, 2) = this.on_path(changed_ind, 2) + delta;

            if delta < 0
                dying_ind = this.on_path(:, 2) < 1;
                dying_id = this.on_path(dying_ind, 1);

                if ~isempty(dying_id)
                    this.on_path(dying_ind, :) = [];
                    this.influence(dying_ind, :) = [];
                    changed_id(ismember(changed_id, dying_id)) = [];
                    add_ind = (length(this.dying) + 1):(length(this.dying) + length(dying_id));
                    this.dying(add_ind, 1) = dying_id;
                    this.dying(add_ind, 2) = this.T_max;
                end

                if ~isempty(changed_id)
                    changed_ind = ismember(this.on_path(:, 1), changed_id);
                    this.calc_influence(changed_ind);
                end

                return
            end

            this.calc_influence(changed_ind);
        end

        function find_best_path_classify(this)

            for i = 1:length(this.path_id)
                ancestor = this.get_ancestor(this.path_id(i));
                this.weight_changed(ancestor, 1);
            end

            this.dying_speed = length(this.path_id);
        end

    end

    methods (Access = public)

        function [node, sample] = get_closest_alive(this)
            len = inf;

            while len > this.neighbor_dist
                sample = this.get_sample();
                this.compare_alive = this.position(this.alive(:, 1), 1:2) - sample(1:2);
                [len, index] = min(sum(this.compare_alive(:, 1:2) .^ 2, 2));
            end

            node = this.position(this.alive(index, 1), :);
        end

        function flag = neighbors_classify(this)

            if this.neighbors()
                flag = true;
                return
            end

            flag = false;
            neighbor_on_path_ind = ismember(this.on_path(:, 1), this.near_nodes.id);

            if any(neighbor_on_path_ind)
                this.new_node.dying_time = max(this.influence(neighbor_on_path_ind, 1)) * this.T_delta +this.T_max;
                return
            end

            dying_neighbor_ind = ismember(this.dying(:, 1), this.near_nodes.id);
            this.new_node.dying_time = mean(this.dying(dying_neighbor_ind, 2));

            if isnan(this.new_node.dying_time)
                a = 0;
            end

        end

        function insert_node_classify(this)
            this.insert_node();

            if this.new_node.dying_time
                this.dying(end + 1, 1) = this.new_node.id;
                this.dying(end, 2) = this.new_node.dying_time;
                this.alive(end + 1) = this.new_node.id;
            end

        end

        function id_rewire = rewire_classify(this)

            if this.near_nodes.num == 1 %no need to rewire
                id_rewire = []; return
            end

            this.compare_near(:, 1) = this.near_nodes.cost_to_newNode(:, 1) + this.new_node.cost_to_root;
            index_rewire = find(this.compare_near(:, 1) < this.near_nodes.cost_to_root(:, 1));
            id_rewire = this.near_nodes.id(index_rewire, 1);

            if isempty(id_rewire)
                return
            end

            [rewire_path_id, ~, ~] = intersect(this.on_path(:, 1), id_rewire);

            if ~isempty(rewire_path_id)
                new_node_ancestor = this.get_ancestor(this.new_node.id);
                this.weight_changed(new_node_ancestor, length(rewire_path_id));

                for i = 1:length(rewire_path_id)
                    near_node_ancestor = this.get_ancestor(this.parent(rewire_path_id(i)));
                    near_node_ancestor(end) = [];
                    this.weight_changed(near_node_ancestor, -1);
                end

            end

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

        function path = find_path_classify(this)
            path = this.find_path();
            new_node_ancestor = this.get_ancestor(this.new_node.id);
            this.weight_changed(new_node_ancestor, 1);
            this.dying_speed = length(this.path_id);
        end

        function start_classify_rrt(this)
            t = tic;
            this.start_direct(t, this.start_dying_step);
            this.find_best_path_classify();
            this.alive = (1:this.node_num)';
            dying_id = this.alive(~ismember(this.alive(:, 1), this.on_path(:, 1)));
            this.dying(1:length(dying_id), 1) = dying_id;
            this.dying(1:length(dying_id), 2) = this.T_max;
            disp('fsdfasdfasdf')

            while this.record(toc(t), this.num_iter)
                this.num_iter = this.num_iter + 1;
                [closest_node, sample] = this.get_closest_alive();
                this.new_node.position(1, :) = this.robot.transfer(sample, closest_node);

                if this.neighbors_classify()
                    continue
                end

                this.num_neighbor = this.num_neighbor + this.near_nodes.num;
                this.choose_parent();

                if this.new_node.id_parent > 0
                    this.insert_node_classify();
                    this.rewire_classify();

                else
                    this.num_no_parent = this.num_no_parent + 1;
                end

                if norm(this.new_node.position(1:3) - this.goal(1:3)) < this.threshold_goal
                    this.find_path_classify();
                end

                if this.is_delete && this.node_num > this.max_nodes
                    this.delete_unuesd_node();
                end

                if this.search_area_rate > 0
                    this.update_step();
                end

                this.tick();
            end

            toc(t)
            [cost, path] = this.find_best_path();
            fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d\n邻居个数%d\n路径代价为%f\n', this.num_iter, this.num_close, this.num_goal, this.num_no_parent, this.num_rewire, this.num_neighbor, cost);
            this.best_path = this.follow_ground(path);
        end

        function init(this)
            conf = this.config_manger_classify.load(this.rand_id);
            this.T_delta = conf.T_delta;
            this.T_max = conf.T_max;
            this.dying_speed = conf.dying_speed;
            this.alpha = conf.alpha;
            this.mini_dying_num = conf.mini_dying_num;
            this.start_dying_step = conf.start_dying_step;
            this.influence = zeros(1, 1, 'single');
            this.on_path = zeros(1, 2, 'uint32');
            this.dying = zeros(1, 2, 'uint32');
            this.dead = zeros(1, 1, 'uint32');
            this.influence(1, :) = [];
            this.on_path(1, :) = [];
            this.dying(1, :) = [];
            this.dead(1, :) = [];
            init@rrt(this);
        end

        function get_new_config(this, config_dir)
            this.config_manger_classify = configs.get_config([this.name_classify, '_', config_dir]);
            get_new_config@rrt(this)
        end

        function this = classify()
            this = this@rrt();
            this.config_manger_classify = configs.get_config(this.name_classify);
        end

    end

end
