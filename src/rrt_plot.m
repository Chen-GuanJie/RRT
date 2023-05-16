classdef rrt_plot < rrt

    properties (SetAccess = private)
        plot_point = []
        path_plot
        replot = []
        edges = []
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

        function display_serachline(this, closest_node, sample, delay_time)

            if delay_time ~= 0
                tp = this.display_line(closest_node, sample, 3, 'k');
                pause(delay_time);
                delete(tp);
            end

        end

        function display_states(this, delay_time)
            % this.display_arrow(this.newNode.position, 10); %
            if delay_time ~= 0
                this.edges(this.new_node.id) = this.display_line(this.position(this.new_node.id_parent, :), this.new_node.position, 1, 'b');
                this.redisplay();
                pause(delay_time);
            end

        end

        function rewire(this)
            ind = rewire@rrt(this);
            replot_num = length(ind);
            this.replot(1:replot_num, 1) = ind;
            this.replot(1:replot_num, 2) = this.new_node.id;
        end

        function redisplay(this)

            if ~isempty(this.replot)

                for i = 1:length(this.replot(:, 1))
                    delete(this.edges(this.replot(i, 1)));
                    s = this.position(this.replot(i, 2), 1:3);
                    e = this.position(this.replot(i, 1), 1:3);
                    this.edges(this.replot(i, 1)) = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', 1, 'Color', 'r');
                end

            end

        end

        function path_evaluate(this, path)

            if nargin == 1
                path = this.best_path;
            end

            path_num = length(path(:, 1));
            % clear gca
            subplot(2, 1, 1)
            path(:, 1:5) = path(:, 1:5) * this.maps.map_scale;
            plot(1:path_num, path(path_num:-1:1, 3), 'LineWidth', 1.5, 'color', 'b', 'DisplayName', '飞机高度'); hold on
            plot(1:path_num, path(path_num:-1:1, 4), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '地形高度');
            legend
            subplot(2, 1, 2)
            % plot(1:path_num, path(path_num:-1:1, 9), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '地形高度');
            % legend
            % subplot(3, 1, 3)
            plot(1:path_num, path(path_num:-1:1, 5), 'LineWidth', 1.5, 'color', 'k', 'DisplayName', '离地高度');
            legend
        end

    end

    methods (Access = public)

        function show_result(this)
            display = this.config_manger.load(this.rand_id).display;
            show_result@benchmark(this);
            display_names = fieldnames(display);

            if utils.in_cell(display_names, 'cutaway')
                utils.get_instance().locate_figure('cutaway', display.cutaway.save_format)
                this.path_evaluate();
                hold off;
            end

            if utils.in_cell(display_names, 'path_best')
                this.show_map();
                this.best_path = this.maps.to_normal_size(this.best_path);
                plot3(this.best_path(:, 1), this.best_path(:, 2), this.best_path(:, 3), 'LineWidth', 2, 'Color', 'g');
                hold off;
            end

        end

        function show_map(this)

            if ~utils.get_instance().locate_figure('main_map')
                this.maps.display_map(); hold on
                % this.plot_point(1).point = scatter3(this.start_point(1), this.start_point(2), this.start_point(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
                % this.plot_point(2).point = scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
                % this.plot_point(1).text = text(this.start_point(1), this.start_point(2), this.start_point(3), '  start');
                % this.plot_point(2).text = text(this.goal(1), this.goal(2), this.goal(3), '  goal');
                xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
                title('RRT算法');
            end

        end

        function show_search_tree(this)
            utils.get_instance().locate_figure('main_map');

            for i = 1:length(this.parent)

                if this.parent(i) > 0
                    this.edges(i) = this.display_line(this.position(this.parent(i), 1:3), this.position(i, 1:3), 1, 'b');
                end

            end

        end

        function delete_search_tree(this)
            delete(this.edges);
        end

        function init(this, rand_config_id)

            if nargin < 2
                this.rand_id = rand;
            else
                this.rand_id = rand_config_id;
            end

            this.replot = [];
            init@rrt(this);
        end

        function this = rrt_plot()
            this = this@rrt();
        end

        function set_start_end(this, s, g)
            set_start_end@rrt(s, g);
            figure(1)
            delete(this.plot_point(1).point);
            delete(this.plot_point(2).point);
            delete(this.plot_point(1).text);
            delete(this.plot_point(2).text);
            this.plot_point(1).point = scatter3(this.start_point(1), this.start_point(2), this.start_point(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k');
            this.plot_point(1).text = text(this.start_point(1), this.start_point(2), this.start_point(3), '  起点');
            this.plot_point(2).point = scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', 'o', 'MarkerEdgeColor', 'k');
            this.plot_point(2).text = text(this.goal(1), this.goal(2), this.goal(3), '  终点');

        end

        function start_rtdisplay(this, delay_time)
            mini_path_len = inf;
            this.show_map();
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
                    this.display_states(delay_time)

                else
                    this.num_no_parent = this.num_no_parent + 1;
                end

                if norm(this.new_node.position(1:3) - this.goal(1:3)) < this.threshold_goal
                    [mini_path_len, path] = this.find_path(mini_path_len);

                    if ~isempty(this.path_plot)
                        delete(this.path_plot);
                    end

                    this.path_plot = plot3(path(:, 1), path(:, 2), path(:, 3), 'LineWidth', 2, 'color', 'g');

                end

            end

            toc(t)
            [cost, path] = this.find_best_path();
            fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d\n邻居个数%d\n路径代价为%f', this.num_iter, this.num_close, this.num_goal, this.num_no_parent, this.num_rewire, this.num_neighbor, cost);
            this.best_path = this.follow_ground(path);
        end

    end

end
