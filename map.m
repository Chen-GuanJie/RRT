classdef map < handle

    properties (SetAccess = public)
        X = zeros(1, 1)
        Y = zeros(1, 1)
        Z = zeros(1, 1)
        X_num = 0
        Y_num = 0
        Z_num = 0
    end

    properties (SetAccess = private)
        ZT = zeros(1, 1) %地图转置
        height_limit = 1
        threshold_high = 1 %离地最高高度
        threshold_low = 1 %离地最低高度
        height_scale = 500 %地图相邻点的距离
        mini_x = 0
        mini_y = 0
        %temp value
        tmp_ind = 0
        h_up = zeros(1, 1)
        h_down = zeros(1, 1)
        tmp_h = zeros(1, 1)
        y_up = zeros(1, 1)
        y_down = zeros(1, 1)
        x_ind = zeros(1, 1)

        % debug
        % debug2
    end

    methods (Access = public)

        function set_height_limit(this, height_limit)
            this.height_limit = height_limit;
        end

        function this = map(dem_data)

            %             if isa(arg, 'str') && exist(arg, 'file')
            % dem_data = conf.dem_data; %coder.load(arg);
            dem_data = sortrows(dem_data);
            %             elseif isa(arg, 'double')
            %                 dem_data = arg;
            %             end

            x = dem_data(:, 1);
            y = dem_data(:, 2);
            z = dem_data(:, 3);
            this.height_scale = 500; %x(2) - x(1);
            this.mini_x = min(x);
            this.mini_y = min(y);

            % x = x - this.mini_x;
            % y = y - this.mini_y;

            [X, Xn] = this.grid(x);
            [Y, Yn] = this.grid(y);
            this.X_num = Xn;
            this.Y_num = Yn;
            this.Z_num = Xn * Yn;
            x_index = this.get_uniqu_index(x);
            [x_size, ~] = size(x);
            x_index(Xn) = x_size;
            x_num = zeros(Xn, 1);
            x_num(1) = x_index(1);
            x_num(2:Xn) = diff(x_index);
            Height = zeros(Xn, Yn);

            for i = 1:1:Xn

                for j = 1:x_num(i)

                    if i == 1
                        start = 1;
                    else
                        start = x_index(i - 1) + 1;
                    end

                    stop = x_index(i);

                    if (stop - start + 1) == Yn
                        Height(i, :) = z(start:stop);
                    else %todo:
                        % a = find(Y == y(start));
                        % b = find(Y == y(stop));

                        % if (b - a) == stop - start
                        %     Height(i, a:b) = z((start:stop));
                        % else
                        m = 1;

                        for k = 1:stop - start + 1

                            while Y(m) ~= y(k)
                                m = m + 1;

                                if m > Yn
                                    break;
                                end

                            end

                            if m > Yn
                                break;
                            end

                            Height(i, m) = z(start + k - 1);
                        end

                        % end

                    end

                end

            end

            this.X = X;
            this.Y = Y;
            this.Z = Height / this.height_scale;
            this.ZT = this.Z';
            this.tmp_h = zeros(1, 10);
        end

        function index = find_closest(this, x, axis)

            if axis == 0
                a = abs(this.X - x);
            else
                a = abs(this.Y - x);
            end

            [~, index] = min(a);
        end

        function [target_loc, delta_dist, flag] = checkPath_v2(this, from, to)
            %检查两点连线是否与地形碰撞
            flag = true;
            delta_dist = norm(from(1:2) - to(1:2));
            start_insdex = zeros(1, 2);
            end_insdex = zeros(1, 2);

            start_insdex(1:2) = round(from(1:2));
            end_insdex(1:2) = round(to(1:2));
            % start_insdex(3) = from(3);
            % end_insdex(3) = to(3);

            % start_insdex = [this.find_closest(start(1), 0), this.find_closest(start(2), 1)];
            % end_insdex = [this.find_closest(endp(1), 0), this.find_closest(endp(2), 1)];

            if end_insdex(1, 1) ~= start_insdex (1, 1)
                k_index = (end_insdex(1, 2) - start_insdex(1, 2)) / (end_insdex(1, 1) - start_insdex(1, 1));
            else
                k_index = inf;
            end

            if abs(k_index) > 1
                y_max = this.X_num;
                x_max = this.Y_num;
                new_Height = this.ZT;
                start_insdex(1, [1 2]) = start_insdex(1, [2 1]);
                end_insdex(1, [1 2]) = end_insdex(1, [2 1]);
                k_index = 1 / k_index;
            else
                y_max = this.Y_num;
                x_max = this.X_num;
                new_Height = this.Z;
            end

            deltaX = end_insdex(1, 1) - start_insdex(1, 1);

            if deltaX > 0
                this.tmp_ind = (0:1:deltaX)';
            elseif deltaX < 0
                this.tmp_ind = (0:-1:deltaX)';
            else
                flag = false;
                target_loc = 0;
                delta_dist = 0;
                return
            end

            num = abs(deltaX) + 1;
            this.h_up = zeros(num, 1);
            this.h_down = zeros(num, 1);
            % this.tmp_h = start_insdex(1, 3) + (end_insdex(1, 3) - start_insdex(1, 3)) * this.tmp_ind / (deltaX); %轨迹高度
            this.y_up = ceil((this.tmp_ind) * k_index + start_insdex(1, 2));
            this.y_down = floor((this.tmp_ind) * k_index + start_insdex(1, 2));
            this.x_ind = start_insdex(1, 1) + this.tmp_ind;
            this.x_ind(this.x_ind > x_max) = x_max;
            this.x_ind(this.x_ind < 1) = 1;

            this.y_up(this.y_up > y_max) = y_max;
            this.y_up(this.y_up < 1) = 1;

            this.y_down(this.y_down < 1) = 1;
            this.y_down(this.y_down > y_max) = y_max;

            for i = 1:1:num
                this.h_up(i, 1) = new_Height(this.x_ind(i), this.y_up(i));
                this.h_down(i, 1) = new_Height(this.x_ind(i), this.y_down(i));
            end

            ground_h = 0.5 .* (this.h_up + this.h_down); %地形高度
            delta_dist = delta_dist / num;
            target_h = this.height_limit + ground_h; %跟踪高度
            target1 = from(1):(to(1) - from(1)) / (num - 1):to(1);
            target2 = from(2):(to(2) - from(2)) / (num - 1):to(2);
            target_loc = [target1', target2', target_h, ground_h];

        end

        function output = find_height(this, node)
            node(1:2) = round(node(1:2));
            x_up = ceil(node(1));
            x_down = floor(node(1));
            up_y = ceil(node(2));
            down_y = floor(node(2));
            output = 0.25 * (this.Z(x_up, up_y) + this.Z(x_up, down_y) + this.Z(x_down, up_y) + this.Z(x_down, down_y));
        end

        function flag = checkPath(this, start_insdex, end_insdex)
            %检查两点连线是否与地形碰撞
            flag = true;
            start_insdex(1:2) = round(start_insdex(1:2));
            end_insdex(1:2) = round(end_insdex(1:2));
            % start_insdex = [this.find_closest(start(1), 0), this.find_closest(start(2), 1)];
            % end_insdex = [this.find_closest(endp(1), 0), this.find_closest(endp(2), 1)];

            if end_insdex(1, 1) ~= start_insdex (1, 1)
                k_index = (end_insdex(1, 2) - start_insdex(1, 2)) / (end_insdex(1, 1) - start_insdex(1, 1));
            else
                k_index = inf;
            end

            if abs(k_index) > 1
                y_max = this.X_num;
                new_Height = this.ZT;
                start_insdex(1, [1 2]) = start_insdex(1, [2 1]);
                end_insdex(1, [1 2]) = end_insdex(1, [2 1]);
                k_index = 1 / k_index;
            else
                y_max = this.Y_num;
                new_Height = this.Z;
            end

            deltaX = end_insdex(1, 1) - start_insdex(1, 1);

            if deltaX > 0
                this.tmp_ind = 0:1:deltaX;

            elseif deltaX < 0
                this.tmp_ind = 0:-1:deltaX;
            else
                return
            end

            this.h_up = zeros(1, abs(deltaX) + 1);
            this.h_down = zeros(1, abs(deltaX) + 1);
            this.tmp_h = start_insdex(1, 3) + (end_insdex(1, 3) - start_insdex(1, 3)) * this.tmp_ind / (deltaX); %轨迹高度
            this.y_up = ceil((this.tmp_ind) * k_index + start_insdex(1, 2));
            this.y_down = floor((this.tmp_ind) * k_index + start_insdex(1, 2));
            this.x_ind = start_insdex(1, 1) + this.tmp_ind;
            this.y_up(this.y_up > y_max) = y_max;
            this.y_down(this.y_down < 1) = 1;

            for i = 1:1:abs(deltaX) + 1

                this.h_up(1, i) = new_Height(this.x_ind(i), this.y_up(i));
                this.h_down(1, i) = new_Height(this.x_ind(i), this.y_down(i));

            end

            ground_h = 0.5 .* (this.h_up + this.h_down); %地形高度
            % target_h = this.height_limit + ground_h; %跟踪高度
            this.tmp_h = this.tmp_h - ground_h; %离地高度
            % too_hight = this.tmp_h > this.threshold_high;
            % too_low = this.tmp_h < this.threshold_low;
            % retval =  k_index target_h;

            % this.tmp_h = (this.tmp_h - target_h) ./ target_h;

            if min(this.tmp_h) < 0
                flag = false;
            end

            % if min(this.tmp_h) < 0.4
            %     flag = true;
            % end

            % if min(this.tmp_h) < 0
            %     % flag = false;
            % else
            %     [h, i] = min(this.tmp_h);
            % end

        end

        function best_path = to_normal_size(this, best_path)
            best_path(:, 1) = best_path(:, 1) + this.mini_x;
            best_path(:, 2) = best_path(:, 2) + this.mini_y;
            best_path(:, 3) = best_path(:, 3);
        end

    end

    methods (Static)

        function x_index = get_uniqu_index(x)
            x_diff = diff(x);
            x_index = find(x_diff ~= 0);
        end

        function [g, num] = grid(x)
            x_unique = unique(x);
            [num, ~] = size(x_unique);
            % num = a(1, 1);
            g = sortrows(x_unique)';
        end

    end

end
