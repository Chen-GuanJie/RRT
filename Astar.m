classdef Astar < handle

    properties (SetAccess = private)
        open = zeros(1, 3) %1:dimension  map  F  G
        open_num = 0;
        closed = zeros(1, 3)
        G = zeros(1, 1)
        H = zeros(1, 1)
        maps %= map([0 0])
        robot %= uav()
        dimension = 2
        neighbors = zeros(1, 3)
        grid_displace = zeros(1, 3)
        start = zeros(1, 3)
        goal = zeros(1, 3)
        tmp_ind
        tmp_value
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

        function init_para(this)
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

            this.grid_displace(0, :) = [];
        end

        function node = find_mini(this)
            [~, ind] = mini(this.open(:, this.dimension + 2));
            node = this.open(ind, :);
            this.open(ind, :) = [];
            this.closed(end + 1, :) = node;
        end

        function operate_neighbors(this, node)
            %处理相邻节点
            this.neighbors = this.grid_displace + node(1, 1:this.dimension);
            this.neighbors(:, this.dimension + 1) = this.mapping(this.neighbors);
            %超出边界的删去
            for i = 1:this.dimension
                this.tmp_ind = this.neighbors(:, this.dimension) < 0 || this.neighbors(:, this.dimension) > this.maps.max_ind(this.dimension);
                this.neighbors(this.tmp_ind, :) = [];
            end

            %closed的点删去
            [~, this.tmp_ind, ~] = intersect(this.neighbors(:, this.dimension + 1), this.closed(:, this.dimension + 1));
            this.neighbors(this.tmp_ind, :) = [];
        end

        function calc_F(this, node)
            this.tmp_value(:, 1) = this.robot.calc_G(this.neighbors, node);
            this.neighbors(:, this.dimension + 3) = this.tmp_value(:, 1) + node(this.dimension + 3); %G
            this.tmp_value(:, 1) = this.robot.calc_displace(this.neighbors, node);
            this.neighbors(:, this.dimension + 2) = this.tmp_value(:, 1) + this.neighbors(:, this.dimension + 3); %F
        end

        function add_neighbor2open(this)
            %将节点加入到open中
            %更新已有节点
            [~, ni, this.tmp_ind] = intersect(this.neighbors(:, this.dimension + 1), this.closed(:, this.dimension + 1));
            si = this.neighbors(ni, this.dimension + 2) < this.closed(this.tmp_ind, this.dimension + 2);
            this.closed(this.tmp_ind(si), this.dimension + 2) = this.neighbors(ni(si), this.dimension + 2);
            this.neighbors(ni(si), :) = [];
            %加入新节点
            this.open(this.open_num:this.open_num + length(this.neighbors(:, 1)) - 1, 1:this.dimension + 3) = this.neighbors(:, 1:this.dimension + 3);
            this.open_num = this.open_num + length(this.neighbors(:, 1));
        end

    end

    methods (Access = public)

        function this = Astar(conf)
            this.maps = map(conf.dem_data);
            this.robot = uav(conf);
            this.dimension = conf.dimension;
            this.open = zeros(1, this.dimension + 1);
            this.closed = zeros(1, this.dimension + 1);
            this.init_para();
        end

        function start_Astar(this)

            while 1
                node = this.find_mini();
                this.operate_neighbors(node);
                this.calc_F(node);
                this.add_neighbor2open();
            end

        end

    end

end
