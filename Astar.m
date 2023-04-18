classdef Astar < handle

    properties (SetAccess = private)
        ind_mapping = 3
        ind_F = 4
        ind_G = 5
        ind_id = 6
        ind_parent = 7
        ind_evaluate = 8
        ind_closed_neighbor_num = 9
        open = zeros(1, 6) %1:dimension  map  F  G id parent evaluate
        open_num = 2
        node_num = 2
        closed = zeros(1, 6) %1:dimension  map  F  G id  parent evaluate closed_neighbor_num
        close_closed = zeros(1, 6)
        all_node = zeros(1, 6) %
        max_neighbor_num = 8
        G = zeros(1, 1)
        H = zeros(1, 1)
        maps %= map([0 0])
        robot %= uav()
        dimension = 2
        neighbors = zeros(1, 3)
        grid_displace = zeros(1, 3)
        start = zeros(1, 3)
        goal = zeros(1, 3)
        node
        calc_G
        calc_H
        rate
    end

    methods (Static)

        function output = mapping(input)
            %高维向低维映射
            n = 1000;
            output = zeros(length(input(:, 1)), 1);

            for i = 1:length(input(1, :))
                output(:, 1) = output(:, 1) + input(:, i) / n;
                n = n * 1000;
            end

        end

    end

    methods (Access = private)

        function output = calc_G_1(this, from, to)
            tmp = to(:, 1:this.dimension) - from(1, 1:this.dimension);
            tmp(:, this.ind_mapping) = this.rate * abs(to(:, this.ind_evaluate) - from(1, this.ind_evaluate));
            tmp = sum(tmp .^ 2, 2);
            output = sqrt(tmp);
        end

        function output = calc_H_1(this, from, to)
            tmp = to(1, 1:this.dimension) - from(:, 1:this.dimension);
            tmp(:, this.ind_mapping) = this.rate * 5 * abs(to(:, this.ind_evaluate) - from(1, this.ind_evaluate));
            tmp = sum(tmp .^ 2, 2);
            output = sqrt(tmp);

        end

        function output = calc_G_2(this, from, to)
            tmp = to(:, 1:this.dimension) - from(1, 1:this.dimension);
            tmp(:, this.dimension) = this.rate * abs(tmp(:, this.dimension));
            tmp = sum(tmp .^ 2, 2);
            output = sqrt(tmp);
        end

        function output = calc_H_2(this, from, to)
            tmp = to(1, 1:this.dimension) - from(:, 1:this.dimension);
            tmp = sum(tmp .^ 2, 2);
            output = sqrt(tmp);
        end

        function init_para(this)
            %生成位移矩阵
            m = 3 ^ this.dimension;
            this.grid_displace = zeros(m, this.dimension);
            n = m / 3;
            s = 1;

            function add_value(this, start, num, i)
                % this.grid_displace(start + 1:start + num, i) = 0;
                this.grid_displace(start + num + 1:start + 2 * num, i) = 1;
                this.grid_displace(start + 2 * num + 1:start + 3 * num, i) = -1;
            end

            for i = 1:this.dimension

                for j = 1:s
                    add_value(this, (j - 1) * 3 * n, n, i);
                end

                s = s * 3;
                n = n / 3;
            end

            this.grid_displace(1, :) = [];
        end

        function find_mini(this)
            [~, ind] = min(this.open(:, this.ind_F));
            this.node = this.open(ind, :);
            this.open(ind, :) = [];
            this.open_num = this.open_num - 1;
        end

        function flag = operate_neighbors(this)
            %处理相邻节点
            this.neighbors = this.grid_displace + this.node(1, 1:this.dimension);
            this.neighbors(:, this.ind_mapping) = this.mapping(this.neighbors);
            %超出边界的删去
            for i = 1:this.dimension
                ind = this.neighbors(:, i) <= 0 | this.neighbors(:, i) >= this.maps.max_ind(i);

                if max(abs(ind)) ~= 0
                    this.neighbors(ind, :) = [];
                end

            end

            %closed的点删去
            [~, ind_n, ind_c] = intersect(this.neighbors(:, this.ind_mapping), this.closed(:, this.ind_mapping));
            this.neighbors(ind_n, :) = [];
            num = length(this.neighbors(:, 1));

            if num > 0
                flag = true;

                for i = 1:num
                    this.neighbors(i, this.ind_evaluate) = this.maps.Z(this.neighbors(i, 1), this.neighbors(i, 2));
                end

                %scatter3(this.neighbors(:, 1), this.neighbors(:, 2), this.neighbors(:, this.ind_evaluate), 1, 'filled', 'o');
                %pause(0.001);

            else
                flag = false;
            end

            %更新closed的邻居数
            this.closed(ind_c, this.ind_closed_neighbor_num) = this.closed(ind_c, this.ind_closed_neighbor_num) + 1; %相邻的closed加1
            this.node(1, this.ind_closed_neighbor_num) = length(ind_c);

            if max(this.closed(:, this.ind_closed_neighbor_num)) == this.max_neighbor_num %周围全是close
                ind = find(this.closed(:, this.ind_closed_neighbor_num) == this.max_neighbor_num);
                this.close_closed(end + 1:end + length(ind), :) = this.closed(ind, :);
                this.closed(ind, :) = [];
            end

        end

        function calc_F(this)
            tmp = this.calc_G(this.node, this.neighbors);
            this.neighbors(:, this.ind_G) = tmp + this.node(this.ind_G); %G
            tmp = this.calc_H(this.neighbors, this.goal);
            this.neighbors(:, this.ind_F) = tmp + this.neighbors(:, this.ind_G); %F
        end

        function flag = add_neighbor2open(this)
            %将节点加入到open中
            %更新已有节点
            [~, ni, ind] = intersect(this.neighbors(:, this.ind_mapping), this.open(:, this.ind_mapping)); %已在open中
            si = this.neighbors(ni, this.ind_F) < this.open(ind, this.ind_F);
            this.open(ind(si), this.ind_F) = this.neighbors(ni(si), this.ind_F);
            this.open(ind(si), this.ind_parent) = this.node(1, this.ind_id);
            this.neighbors(ni, :) = [];
            %加入新节点
            num = length(this.neighbors(:, 1));
            this.neighbors(:, this.ind_id) = (this.node_num:this.node_num + num - 1)'; %id
            this.neighbors(:, this.ind_parent) = this.node(1, this.ind_id); %parent
            this.open(this.open_num:(this.open_num + num - 1), 1:this.ind_evaluate) = this.neighbors(:, 1:this.ind_evaluate); %添加到open
            this.all_node(this.node_num:(this.node_num + num - 1), :) = this.neighbors(:, 1:this.ind_evaluate);
            this.open_num = this.open_num + num;
            this.node_num = this.node_num + num;
            this.closed(end + 1, :) = this.node;

            if max(abs(this.node(1, 1:this.dimension) - this.goal(1, 1:this.dimension))) < 2
                flag = false;
            else
                flag = true;
            end

        end

        function output = show_path(this)
            ind = 1;
            output = [];
            output(ind, :) = this.goal;
            prev = this.node(1, this.ind_id);

            while prev > 0
                ind = ind + 1;
                output(ind, :) = this.all_node(prev, :);
                prev = this.all_node(prev, this.ind_parent);
            end

        end

    end

    methods (Access = public)

        function this = Astar(conf)
            this.maps = map.get_instance(conf.dem_data);
            % this.robot = uav(conf);
            this.set_params(conf);
        end

        function set_start_end(this, s, g)
            this.start = s(1, 1:this.dimension);
            this.goal = g(1, 1:this.dimension);
            this.start(1, this.ind_evaluate) = this.maps.Z(this.start(1, 1), this.start(1, 2));
            this.goal(1, this.ind_evaluate) = this.maps.Z(this.goal(1, 1), this.goal(1, 2));

        end

        function set_params(this, conf)
            this.dimension = conf.dimension;
            this.ind_mapping = this.dimension + 1;
            this.ind_F = this.dimension + 2;
            this.ind_G = this.dimension + 3;
            this.ind_id = this.dimension + 4;
            this.ind_parent = this.dimension + 5;
            this.ind_evaluate = this.dimension + 6;
            this.ind_closed_neighbor_num = this.dimension + 7;
            this.init_para();
            this.max_neighbor_num = 3 ^ this.dimension - 1;
            this.rate = conf.rate;
            this.start = conf.start(1, 1:this.dimension);
            this.goal = conf.goal(1, 1:this.dimension);
            this.start(1, this.ind_evaluate) = this.maps.Z(this.start(1, 1), this.start(1, 2));
            this.goal(1, this.ind_evaluate) = this.maps.Z(this.goal(1, 1), this.goal(1, 2));
            this.open_num = 2;
            this.node_num = 2;

            if this.dimension == 2
                this.calc_G = @this.calc_G_1;
                this.calc_H = @this.calc_H_1;

            elseif this.dimension == 3
                this.calc_G = @this.calc_G_2;
                this.calc_H = @this.calc_H_2;

            end

        end

        function output = start_Astar(this)
            flag = true;
            this.open = zeros(1, this.ind_evaluate);
            this.close_closed = zeros(1, this.ind_closed_neighbor_num);
            this.closed = zeros(1, this.ind_closed_neighbor_num);
            this.all_node = zeros(1, this.ind_evaluate);
            this.start(1, this.ind_id:this.ind_parent) = [1 -1];
            this.open(1, :) = this.start; %0 0 0 1 -1 0;
            this.all_node(1, :) = this.start;
            % figure(1);
            % meshz(1:this.maps.X_num, 1:this.maps.Y_num, this.maps.Z'); hold on
            tic
            tmp = zeros(1, 1);
            inde = 1;
            lastime = 0;

            while flag
                this.find_mini();

                if toc - lastime > 0.3
                    lastime = toc;
                    tmp(inde, 1) = length(this.all_node(:, 1));
                    inde = inde + 1;

                end

                if this.operate_neighbors()
                    this.calc_F();
                    flag = this.add_neighbor2open();
                else
                    flag = true;
                end

            end

            toc
            output = this.show_path();
            % plot3(output(:, 1), output(:, 2), output(:, this.ind_evaluate), 'LineWidth', 2, 'color', 'g');
            % scatter3(this.open(:, 1), this.open(:, 2), this.open(:, this.ind_evaluate), 1, 'filled', 'color', 'r');
            % scatter3(this.closed(:, 1), this.closed(:, 2), this.closed(:, this.ind_evaluate), 1, 'filled', 'color', 'b');
            % figure(2)
            % plot(tmp);
            % plot(diff(tmp))

        end

    end

end
