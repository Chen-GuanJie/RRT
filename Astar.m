classdef Astar < handle

    properties (SetAccess = private)
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
            tmp(:, this.dimension + 1) = this.rate * abs(to(:, this.dimension + 6) - from(1, this.dimension + 6));
            tmp = sum(tmp .^ 2, 2);
            output = sqrt(tmp);
        end

        function output = calc_H_1(this, from, to)
            tmp = to(1, 1:this.dimension) - from(:, 1:this.dimension);
            tmp(:, this.dimension + 1) = this.rate*5 * abs(to(:, this.dimension + 6) - from(1, this.dimension + 6));
            tmp = sum(tmp .^ 2, 2);
            output = sqrt(tmp);

        end

        %
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
            [~, ind] = min(this.open(:, this.dimension + 2));
            this.node = this.open(ind, :);
            this.open(ind, :) = [];
            this.open_num = this.open_num - 1;
        end

        function flag = operate_neighbors(this)
            %处理相邻节点
            this.neighbors = this.grid_displace + this.node(1, 1:this.dimension);
            this.neighbors(:, this.dimension + 1) = this.mapping(this.neighbors);
            %超出边界的删去
            for i = 1:this.dimension
                ind = this.neighbors(:, i) <= 0 | this.neighbors(:, i) >= this.maps.max_ind(i);

                if max(abs(ind)) ~= 0
                    this.neighbors(ind, :) = [];
                end

            end

            %closed的点删去
            [~, ind_n, ind_c] = intersect(this.neighbors(:, this.dimension + 1), this.closed(:, this.dimension + 1));
            this.neighbors(ind_n, :) = [];
            num = length(this.neighbors(:, 1));

            if num > 0
                flag = true;

                for i = 1:num
                    this.neighbors(i, this.dimension + 6) = this.maps.Z(this.neighbors(i, 1), this.neighbors(i, 2));
                end

            else
                flag = false;
            end

            % scatter3(this.neighbors(:, 1), this.neighbors(:, 2), this.neighbors(:, this.dimension + 6), 1, 'filled', 'o');
            % pause(0.001);
            %更新closed的邻居数
            this.closed(ind_c, this.dimension + 7) = this.closed(ind_c, this.dimension + 7) + 1; %相邻的closed加1
            this.node(1, this.dimension + 7) = length(ind_c);

            if max(this.closed(:, this.dimension + 7)) == this.max_neighbor_num %周围全是close
                ind = find(this.closed(:, this.dimension + 7) == this.max_neighbor_num);
                this.close_closed(end + 1:end + length(ind), :) = this.closed(ind, :);
                this.closed(ind, :) = [];
            end

        end

        function calc_F(this)
            tmp = this.calc_G(this.node, this.neighbors);
            this.neighbors(:, this.dimension + 3) = tmp + this.node(this.dimension + 3); %G
            tmp = this.calc_H(this.neighbors, this.goal);
            this.neighbors(:, this.dimension + 2) = tmp + this.neighbors(:, this.dimension + 3); %F
        end

        function flag = add_neighbor2open(this)
            %将节点加入到open中
            %更新已有节点
            [~, ni, ind] = intersect(this.neighbors(:, this.dimension + 1), this.open(:, this.dimension + 1)); %已在open中
            si = this.neighbors(ni, this.dimension + 2) < this.open(ind, this.dimension + 2);
            this.open(ind(si), this.dimension + 2) = this.neighbors(ni(si), this.dimension + 2);
            this.open(ind(si), this.dimension + 5) = this.node(1, this.dimension + 4);
            this.neighbors(ni, :) = [];
            %加入新节点
            num = length(this.neighbors(:, 1));
            this.neighbors(:, this.dimension + 4) = (this.node_num:this.node_num + num - 1)'; %id
            this.neighbors(:, this.dimension + 5) = this.node(1, this.dimension + 4); %parent
            this.open(this.open_num:(this.open_num + num - 1), 1:this.dimension + 6) = this.neighbors(:, 1:this.dimension + 6); %添加到open
            this.all_node(this.node_num:(this.node_num + num - 1), :) = this.neighbors(:, 1:this.dimension + 6);
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
            prev = this.node(1, this.dimension + 4);

            while prev > 0
                ind = ind + 1;
                output(ind, :) = this.all_node(prev, :);
                prev = this.all_node(prev, this.dimension + 5);
            end

        end

    end

    methods (Access = public)

        function this = Astar(conf)
            this.maps = map.get_instance(conf.dem_data);
            % this.robot = uav(conf);
            this.set_params(conf);
        end

        function set_params(this, conf)
            this.dimension = conf.dimension;
            this.open = zeros(1, this.dimension + 1);
            this.closed = zeros(1, this.dimension + 1);
            this.init_para();
            this.max_neighbor_num = 3 ^ this.dimension - 1;
            this.rate = conf.rate;
            this.start = conf.start(1, 1:this.dimension);
            this.goal = conf.goal(1, 1:this.dimension);
            this.start(1, this.dimension + 6) = this.maps.Z(this.start(1, 1), this.start(1, 2));
            this.goal(1, this.dimension + 6) = this.maps.Z(this.goal(1, 1), this.goal(1, 2));
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
            this.open = zeros(1, this.dimension + 6);
            this.close_closed = zeros(1, this.dimension + 7);
            this.closed = zeros(1, this.dimension + 7);
            this.all_node = zeros(1, this.dimension + 6);
            this.start(1, this.dimension + 4:this.dimension + 5) = [1 -1];
            this.open(1, :) = this.start; %0 0 0 1 -1 0;
            this.all_node(1, :) = this.start;
            figure(1);
            meshz(1:this.maps.X_num, 1:this.maps.Y_num, this.maps.Z'); hold on
            tic

            while flag
                this.find_mini();

                if this.operate_neighbors()
                    this.calc_F();
                    flag = this.add_neighbor2open();
                else
                    flag = true;
                end

            end

            toc
            output = this.show_path();
            plot3(output(:, 1), output(:, 2), output(:, this.dimension + 6), 'LineWidth', 2, 'color', 'g');
            scatter3(this.open(:, 1), this.open(:, 2), this.open(:, this.dimension + 6), 'color', 'r');
            scatter3(this.closed(:, 1), this.closed(:, 2), this.closed(:, this.dimension + 6), 'color', 'b');
        end

    end

end
