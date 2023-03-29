classdef rrt_plot < rrt

    properties (SetAccess = private)
        plot_point = []
        % replot_num
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

        function rewire_v2(this, new_parent_id)
            rewire_v2@rrt(this, new_parent_id);
            this.replot(1:this.replot_num, 1) = this.tmp_ind;
            this.replot(1:this.replot_num, 2) = this.newNode(9);
        end
%{
        function [path_len, path_num] = trace_back(this, id)
            [path_len, path_num] = trace_back@rrt(this, id);

                        if ~isempty(this.path_plot)
                            delete(this.path_plot);
                        end
                        this.path_plot = plot3(this.path(1:this.tmp_ind, 1), this.path(1:this.tmp_ind, 2), this.path(1:this.tmp_ind, 3), 'LineWidth', 2, 'color', 'g');

        end
%}
        function redisplay(this)

            for i = 1:this.replot_num
                delete(this.edges(this.replot(i, 1)));
                s = this.tree(this.replot(i, 2), 1:3);
                e = this.tree(this.replot(i, 1), 1:3);
                this.edges(this.replot(i, 1)) = plot3([s(1); e(1)], [s(2); e(2)], [s(3); e(3)], 'LineWidth', 1, 'Color', 'r');
            end

        end

        function path_evaluate(this, path_num)
            figure(2)
            clear gca
            subplot(3, 1, 1)
            this.path(:, 1:3) = this.path(:, 1:3) * this.map_scale;
            this.path(:, 8:9) = this.path(:, 8:9) * this.map_scale;
            plot(1:path_num, this.path(path_num:-1:1, 3), 'LineWidth', 1.5, 'color', 'b', 'DisplayName', '飞机高度'); hold on
            plot(1:path_num, this.path(path_num:-1:1, 9), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '地形高度');
            legend
            subplot(3, 1, 2)
            plot(1:path_num, this.path(path_num:-1:1, 9), 'LineWidth', 1.5, 'color', 'r', 'DisplayName', '地形高度');
            legend
            subplot(3, 1, 3)
            plot(1:path_num, this.path(path_num:-1:1, 8), 'LineWidth', 1.5, 'color', 'k', 'DisplayName', '离地高度');
            legend
        end

        function display_map(this)
            meshz(1:this.maps.X_num, 1:this.maps.Y_num, this.maps.Z'); hold on
        end

    end

    methods (Access = public)

        function this = rrt_plot(conf)
            this = this@rrt(conf);
            figure(1)
            this.display_map()
            this.plot_point(1).point = scatter3(this.start(1), this.start(2), this.start(3), 80, "cyan", 'filled', 'o', 'MarkerEdgeColor', 'k'); hold on
            this.plot_point(2).point = scatter3(this.goal(1), this.goal(2), this.goal(3), 80, "magenta", 'filled', "o", 'MarkerEdgeColor', 'k');
            this.plot_point(1).text = text(this.start(1), this.start(2), this.start(3), '  起点');
            this.plot_point(2).text = text(this.goal(1), this.goal(2), this.goal(3), '  终点');
            xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
            title('RRT算法UAV航迹规划路径');

        end

        function output = start_star(this, max_time)
            [output, interp_num] = start_star@rrt(this, max_time);
            figure(1);
            plot3(this.path(1:interp_num, 1), this.path(1:interp_num, 2), this.path(1:interp_num, 3), 'LineWidth', 2, 'color', 'g');
            this.path_evaluate(interp_num);
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

            this.tree = zeros(this.max_nodes, 10);
            this.path = zeros(500, 10);
            this.nodenum = 1;
            this.informed = false;
            this.randnum = this.randnums(1, 1);
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
            path_id = zeros(5, 1);
            this.newNode = [this.start 0 0 0 0];
            this.insert_node(-1);
            neighbor_dist = 20 ^ 2;
            tic

            while toc <= max_time
                this.search_num = this.search_num + 1;
                sample = this.get_sample();
                [closestNode, ~] = this.get_closest(sample);
                this.newNode = this.robot.transfer_directly(sample, closestNode, this.maps.Z);
                this.display_serachline(closestNode, sample, delay_time);

                if this.neighbors(neighbor_dist)
                    this.tooclose = this.tooclose + 1;
                    continue
                end

                new_id = this.choose_parent_v2();

                if norm(this.newNode(1:3) - this.goal(1:3)) > this.threshold_goal

                    if new_id > 0
                        this.insert_node(new_id);
                        this.rewire_v2(new_id);
                        this.display_states(delay_time)

                    else
                        this.no_parent = this.no_parent + 1;
                    end

                else %if flag == 2
                    this.isgoal = this.isgoal + 1;
                    path_id(this.isgoal, 1) = new_id;
                    this.randnum = this.randnums(1, 2); %搜索点不取goal
                    [path_len, ~] = this.trace_back(new_id);

                    if ~isempty(this.path_plot)
                        delete(this.path_plot);
                    end

                    this.path_plot = plot3(this.path(1:this.tmp_ind, 1), this.path(1:this.tmp_ind, 2), this.path(1:this.tmp_ind, 3), 'LineWidth', 2, 'color', 'g');

                    this.prepare_informed(path_len);
                end

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
            this.path_evaluate(interp_num);
            output = this.maps.to_normal_size(this.path);

        end

    end

end
