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

        function path_evaluate(this, bp)

            if nargin == 1
                bp = this.best_path;
            end

            x_data = diff(bp);
            x_data = sqrt(x_data(:, 1) .^ 2 + x_data(:, 2) .^ 2);
            x_data = [0; x_data];
            x_data = cumsum(x_data);
            x_data = x_data * this.maps.map_scale;
            bp(:, 1:5) = bp(:, 1:5) * this.maps.map_scale;
            subplot(2, 1, 1)
            plot(x_data, bp(end:-1:1, 3), 'LineWidth', 1.5, 'color', 'b', 'DisplayName', 'aircraft altitude'); hold on
            plot(x_data, bp(end:-1:1, 4), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', 'topographic height');
            ylabel('height(m)');
            xlabel('distance(m)');
            xlim([x_data(1) x_data(end)]);
            hold off
            legend('Location', 'best');
            subplot(2, 1, 2)
            plot(x_data, bp(end:-1:1, 5), 'LineWidth', 1.5, 'color', 'b'); %,'DisplayName', 'Ground Clearance');
            xlabel('height(m)');
            ylabel('Ground Clearance(m)');
            xlim([x_data(1) x_data(end)]);
            % legend('Location', 'best');
        end

        function draw_angle(this)
            x_data = diff(this.best_path);
            x_data = sqrt(x_data(:, 1) .^ 2 + x_data(:, 2) .^ 2);
            x_data = [0; x_data];
            x_data = cumsum(x_data);
            x_data = x_data * this.maps.map_scale;

            subplot(2, 1, 1);
            n = length(this.best_path);
            angle = zeros(n - 1, 2);

            for i = n:-1:2
                v = this.best_path(i - 1, 1:2) - this.best_path(i, 1:2);
                angle(n - i + 1, 1) = atan2(v(2), v(1));
                angle(n - i + 1, 2) = atan2((this.best_path(i - 1, 3) - this.best_path(i, 3)), norm(v));
            end

            angle = angle .* 180 ./ pi;
            angle = [angle(1, :); angle];
            angle = diff(angle);
            angle = [angle(1, :); angle];
            plot(x_data, angle(:, 2)); %,'DisplayName', 'pitch angle');
            ylabel('pitch angle');
            xlabel('distance(m)');
            xlim([x_data(1) x_data(end)]);
            % legend('Location', 'best')
            subplot(2, 1, 2)
            plot(x_data, angle(:, 1)); %,'DisplayName', 'course');
            xlabel('distance(m)');
            ylabel('yaw angle');
            xlim([x_data(1) x_data(end)]);
            % legend('Location', 'best')
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
                set(gcf, 'unit', 'centimeters', 'position', [3 5 17 8])
                hold off;
            end

            if utils.in_cell(display_names, 'path_best')
                utils.get_instance().locate_figure('path_best', display.path_best.save_format)
                this.show_map(display.path_best.map_interval, display.path_best.normal_map);

                if display.path_best.normal_map
                    bp = this.maps.to_normal_size(this.best_path);
                    y_lim = [this.maps.Y(1) this.maps.Y(end)];
                    x_lim = [this.maps.X(1) this.maps.X(end)];
                else
                    bp = this.best_path;
                    y_lim = [1 this.maps.Y_num];
                    x_lim = [1 this.maps.X_num];
                end

                plot3(bp(:, 1), bp(:, 2), bp(:, 3), 'LineWidth', 1, 'Color', 'red');
                view(2); axis equal;
                ylim(y_lim); xlim(x_lim);
                d = this.maps.X_num / this.maps.Y_num;
                set(gcf, 'unit', 'centimeters', 'position', [3 5 11 12 * d])

                if this.maps.Y(end) > this.maps.X(end)
                    view(-90, 90);
                end

                hold off;
            end

            if utils.in_cell(display_names, 'angle')
                utils.get_instance().locate_figure('angle', display.angle.save_format)
                this.draw_angle();
                set(gcf, 'unit', 'centimeters', 'position', [3 5 17 8])
            end

            if utils.in_cell(display_names, 'search_tree')

                for i = 1:length(display.search_tree.save_index)
                    ind = display.search_tree.save_index{i}(1);
                    utils.get_instance().locate_figure(['search tree ' num2str(ind)], display.search_tree.save_format)
                    this.show_map(display.search_tree.map_interval, display.search_tree.normal_map);

                    if display.search_tree.normal_map
                        this.show_search_tree( ...
                            this.maps.to_normal_size(this.states{ind, 1}.position), ...
                            this.states{ind, 1}.parent);
                        y_lim = [this.maps.Y(1) this.maps.Y(end)];
                        x_lim = [this.maps.X(1) this.maps.X(end)];

                    else
                        this.show_search_tree(this.states{ind, 1}.position, this.states{ind, 1}.parent);
                        y_lim = [1 this.maps.Y_num];
                        x_lim = [1 this.maps.X_num];

                    end

                    view(2); axis equal;
                    ylim(y_lim); xlim(x_lim);
                    d = this.maps.X_num / this.maps.Y_num;
                    set(gcf, 'unit', 'centimeters', 'position', [3 5 20.5 20.5 * d]);

                    if this.maps.Y(end) > this.maps.X(end)
                        view(-90, 90);
                    end

                    hold off
                end

            end

        end

        function show_tree(this)

        end

        function show_map(this, interval, normal_map)

            if nargin < 3
                interval = 5;
                normal_map = true;
            end

            % if ~utils.get_instance().locate_figure('main_map')
            this.maps.display_map(interval, normal_map); hold on

            if normal_map
                s = this.maps.to_normal_size(this.start_point);
                g = this.maps.to_normal_size(this.goal);
            else
                s = this.start_point;
                g = this.goal;
            end

            this.plot_point(1).point = scatter3(s(1), s(2), s(3), 35, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
            this.plot_point(2).point = scatter3(g(1), g(2), g(3), 35, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
            this.plot_point(1).text = text(s(1), s(2), s(3) + 500, '  start');
            this.plot_point(2).text = text(g(1), g(2), g(3) + 500, '  goal');
            ylabel('x(m)'); xlabel('y(m)'); zlabel('z(m)');
            % title('RRT算法');
            % end

        end

        function show_search_tree(this, position_data, parent_data)

            if nargin < 3
                position_data = this.position;
                parent_data = this.parent;
            end

            for i = 1:length(parent_data)

                if parent_data(i) > 0
                    this.display_line(position_data(parent_data(i), 1:3), position_data(i, 1:3), 0.5, 'b'); hold on
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
