classdef rrt_plot < rrt

    properties (SetAccess = private)
        plot_point = []
        path_plot
        ifdisplay = false
        replot = zeros(50, 3)
        edges
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
                this.edges(this.new_node.id) = this.display_line(this.tree(this.new_node.id_parent, :), this.new_node.position, 1, 'b');
                this.redisplay();
                pause(delay_time);
            end

        end

        function rewire_v2(this)
            ind = rewire_v2@rrt(this);
            replot_num = length(ind);
            this.replot(1:replot_num, 1) = ind;
            this.replot(1:replot_num, 2) = this.new_node.id;
        end

        function redisplay(this)

            for i = 1:length(this.replot(:, 1))
                delete(this.edges(this.replot(i, 1)));
                s = this.tree(this.replot(i, 2), 1:3);
                e = this.tree(this.replot(i, 1), 1:3);
                this.edges(this.replot(i, 1)) = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', 1, 'Color', 'r');
            end

        end

        function path_evaluate(this, best_path)
            path_num = length(best_path(:, 1));
            figure(2)
            % clear gca
            subplot(3, 1, 1)
            best_path(:, 1:3) = best_path(:, 1:3) * this.map_scale;
            best_path(:, 8:9) = best_path(:, 8:9) * this.map_scale;
            plot(1:path_num, best_path(path_num:-1:1, 3), 'LineWidth', 1.5, 'color', 'b', 'DisplayName', '飞机高度'); hold on
            plot(1:path_num, best_path(path_num:-1:1, 9), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '地形高度');
            legend
            subplot(3, 1, 2)
            % plot(1:path_num, best_path(path_num:-1:1, 9), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '地形高度');
            % legend
            % subplot(3, 1, 3)
            plot(1:path_num, best_path(path_num:-1:1, 8), 'LineWidth', 1.5, 'color', 'k', 'DisplayName', '离地高度');
            legend
        end

        function display_map(this)
            meshz(1:this.maps.X_num, 1:this.maps.Y_num, this.maps.Z'); hold on
        end

    end

    methods (Access = public)

        function show_map(this)
            figure(1)
            this.display_map()
            this.plot_point(1).point = scatter3(this.start(1), this.start(2), this.start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
            this.plot_point(2).point = scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
            this.plot_point(1).text = text(this.start(1), this.start(2), this.start(3), '  起点');
            this.plot_point(2).text = text(this.goal(1), this.goal(2), this.goal(3), '  终点');
            xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
            title('RRT算法UAV航迹规划路径');

        end

        function show_search_tree(this)

            for i = 1:length(this.parent)

                if this.parent(i) > 0
                    this.edges(i) = this.display_line(this.tree(this.parent(i), 1:3), this.tree(i, 1:3), 1, 'b');
                end

            end

        end

        function delete_search_tree(this)

            delete(this.edges);

        end

        function this = rrt_plot(conf)
            this = this@rrt(conf);
        end

        function output = start_star_1_plot(this, max_time)
            output = this.start_star(max_time);
            this.show_map();
            figure(1);
            output = output(:, 1:3);
            output = [this.start(1:3); output; this.goal(1:3)];
            plot3(output(:, 1), output(:, 2), output(:, 3), 'LineWidth', 2, 'color', 'g');
            %this.path_evaluate(output);
            this.show_search_tree()
        end

        function set_start_end(this, s, g)
            set_start_end@rrt(s, g);
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

        function output = start_star_plot(this, max_time, delay_time)

            if nargin < 2
                max_time = 7;
                delay_time = 0.01;
            end

            this.tree = zeros(this.max_nodes, 3);
            this.node_num = 1;
            this.informed = false;
            this.rand_num = this.rand_nums(1, 1);
            this.search_num = 0;
            this.tooclose = 0;
            this.isgoal = 0;
            this.no_parent = 0;
            this.rewire_num = 0;
            figure(1);

            if ~isempty(this.edges)
                delete(this.edges);
            end

            this.edges = matlab.graphics.chart.primitive.Line(this.max_nodes);
            this.new_node.position = this.start(1, 1:3);
            this.new_node.id_parent = -1;
            this.new_node.cost_to_parent = 0;
            this.insert_node();
            this.show_map();
            mini_path_len = inf;

            tic

            while toc <= max_time
                this.search_num = this.search_num + 1;
                sample = this.get_sample();
                [closest_node, ~] = this.get_closest(sample);
                this.new_node.position(1, :) = this.robot.transfer_directly(sample, closest_node);
                this.display_serachline(closest_node, sample, delay_time);

                if this.neighbors()
                    this.tooclose = this.tooclose + 1;
                    continue
                end

                this.choose_parent_v2();

                if norm(this.new_node.position(1:3) - this.goal(1:3)) > this.threshold_goal

                    if this.new_node.id_parent > 0
                        this.insert_node();
                        this.rewire_v2();
                        this.display_states(delay_time)

                    else
                        this.no_parent = this.no_parent + 1;
                    end

                else %if flag == 2
                    this.isgoal = this.isgoal + 1;
                    this.insert_node();
                    this.path_id(this.isgoal, 1) = this.new_node.id;
                    this.rand_num = this.rand_nums(1, 2); %搜索点不取goal
                    path = this.trace_back(this.new_node.id);
                    path = [this.goal(1:3); path; this.start(1:3)];
                    tmp_value = path(2:end, 1:3) - path(1:end - 1, 1:3);
                    path_len = sum(sqrt(sum(tmp_value .^ 2, 2)));

                    if ~isempty(this.path_plot)
                        delete(this.path_plot);
                    end

                    this.path_plot = plot3(path(:, 1), path(:, 2), path(:, 3), 'LineWidth', 2, 'color', 'g');

                    if mini_path_len > path_len
                        mini_path_len = path_len;
                        this.prepare_informed(path_len);
                    end

                end

                % if this.nodenum > this.max_nodes
                %     this.delete_unuesd_node();
                % end

            end

            if delay_time == 0
                this.show_search_tree();
            end

            % [~, path_number] = size(this.path_id);
            [c, path] = this.find_best_path();
            fprintf('一共搜索%d个点\n相邻过近的点个数%d\n延申到目标点个数%d\n未找到父节点个数%d\n重连个数%d', this.search_num, this.tooclose, this.isgoal, this.no_parent, this.rewire_num);
            fprintf('\n路径代价为%f', c);
            % interp_num = this.interpolation(path_num);
            new_path = this.follow_ground(path);
            % new_path = path;

            if ~isempty(this.path_plot)
                delete(this.path_plot);
            end

            plot3(new_path(:, 1), new_path(:, 2), new_path(:, 3), 'LineWidth', 2, 'color', 'g');
            this.path_evaluate(new_path);
            output = this.maps.to_normal_size(new_path);

        end

    end

end
