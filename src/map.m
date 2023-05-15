classdef map < handle

    properties (SetAccess = public)
        X = zeros(1, 1)
        Y = zeros(1, 1)
        Z = zeros(1, 1)
        X_num = 0
        Y_num = 0
        Z_num = 0
        start_point
        goal
        map_scale
        height_limit
        max_ind = zeros(1, 3)
    end

    properties (SetAccess = private)
        name = 'map'
        config_manger
        rand_id = -1
        map_path = './data/map/'
        map_name = ''
        ZT = zeros(1, 1) %地图转置
        threshold_high = 1 %离地最高高度
        threshold_low = 1 %离地最低高度
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
    end

    methods (Access = private)

        function this = map()
            this.config_manger = configs.get_config(this.name);
        end

        function save_built_map(this, X, Y, Z)
            conf = this.config_manger.load(this.rand_id);
            file_format = strsplit(conf.save_built_map);

            if utils.in_cell(file_format, 'mat')
                save([this.map_path, this.map_name, '_x.mat'], 'X');
                save([this.map_path, this.map_name, '_y.mat'], 'Y');
                save([this.map_path, this.map_name, '_z.mat'], 'Z');
            end

        end

        function output = validate_map(this)
            output = false;

            if length(this.X) == this.X_num && length(this.Y) == this.Y_num && ...
                    this.X_num == size(this.Z, 1) && this.Y_num == size(this.Z, 2)
                output = true;
            end

        end

        function build_map(this, dem_data)
            dem_data = sortrows(dem_data);
            x = dem_data(:, 1);
            y = dem_data(:, 2);
            z = dem_data(:, 3);
            this.mini_x = min(x);
            this.mini_y = min(y);

            % x = x - this.mini_x;
            % y = y - this.mini_y;

            [X_data, Xn] = this.grid(x);
            [Y_data, Yn] = this.grid(y);
            this.X_num = Yn;
            this.Y_num = Xn;
            this.Z_num = Xn * Yn;
            x_index = this.get_uniqu_index(x);
            [x_size, ~] = size(x);
            x_index(Xn) = x_size;
            x_num = zeros(Xn, 1);
            x_num(1) = x_index(1);
            x_num(2:Xn) = diff(x_index);
            Height = zeros(Xn, Yn);
            this.max_ind(1, 1:3) = [Xn, Yn, inf];

            if this.Z_num ~= x_size

                for i = 1:1:Xn

                    for j = 1:x_num(i)

                        if i == 1
                            start_ind = 1;
                        else
                            start_ind = x_index(i - 1) + 1;
                        end

                        stop = x_index(i);

                        if (stop - start_ind + 1) == Yn
                            Height(i, :) = z(start_ind:stop);
                        else %todo:
                            a = find(Y_data == y(start_ind));
                            b = find(Y_data == y(stop));

                            if (b - a) == stop - start_ind
                                Height(i, a:b) = z((start_ind:stop));
                            else
                                m = 1;

                                for k = 1:stop - start_ind + 1

                                    while Y_data(m) ~= y(k)
                                        m = m + 1;

                                        if m > Yn
                                            break;
                                        end

                                    end

                                    if m > Yn
                                        break;
                                    end

                                    Height(i, m) = z(start_ind + k - 1);
                                end

                            end

                        end

                    end

                end

            else

                for i = 1:1:Xn
                    Height(i, :) = z((i - 1) * Yn + 1:i * Yn);
                end

            end

            this.X = Y_data;
            this.Y = X_data;
            this.map_scale = this.X(2) - this.X(1);
            this.save_built_map(this.X, this.Y, Height);
            this.Z = Height / this.map_scale;
            this.ZT = this.Z';
            this.tmp_h = zeros(1, 10);
        end

    end

    methods (Access = public)

        function init(this)
            this.rand_id = rand;
            conf = this.config_manger.load(this.rand_id);
            this.start_point = cell2mat(conf.start_point);
            this.goal = cell2mat(conf.goal);
            conf.map_name = char(conf.map_name);
            this.start_point([1 2]) = this.start_point([2 1]);
            this.goal([1 2]) = this.goal([2 1]);

            if ~strcmp(this.map_name, conf.map_name) || ~this.validate_map()
                this.map_name = conf.map_name;
                n = [this.map_path, this.map_name];

                if (isfile([n, '_x.mat']) || isfile([n, '_x.csv'])) && ...
                        (isfile([n, '_y.mat']) || isfile([n, '_y.csv'])) && ...
                        (isfile([n, '_z.mat']) || isfile([n, '_z.csv']))
                    this.X = utils.load_file(this.map_path, [this.map_name, '_x']);
                    this.map_scale = this.X(2) - this.X(1);
                    this.Y = utils.load_file(this.map_path, [this.map_name, '_y']);
                    this.Z = utils.load_file(this.map_path, [this.map_name, '_z']) / this.map_scale;
                    this.ZT = this.Z';
                    this.X_num = length(this.X);
                    this.Y_num = length(this.Y);
                    this.Z_num = size(this.Z, 1) * size(this.Z, 2);

                else
                    this.build_map(utils.load_file(this.map_path, this.map_name));
                end

            end

            this.height_limit = conf.height_limit / this.map_scale;
        end

        function index = find_closest(this, x, axis)

            if axis == 0
                a = abs(this.this.X - x);
            else
                a = abs(this.this.Y - x);
            end

            [~, index] = min(a);
        end

        function [ground_h, flag] = checkPath_v3(this, from, to)
            flag = true;
            start_insdex = round(from(1:2));
            end_insdex = round(to(1:2));

            if all (start_insdex == end_insdex) || any(start_insdex < 1) || ...
                    any(end_insdex < 1) || start_insdex(1) > this.Y_num || ...
                    end_insdex(1) > this.Y_num || start_insdex(2) > this.X_num || ...
                    end_insdex(1) > this.X_num
                ground_h = [];
                flag = false;
                return
            end

            ind_s = sub2ind([this.Y_num, this.X_num], start_insdex(1), start_insdex(2));
            x_len = end_insdex(2) - start_insdex(2);

            if x_len ~= 0
                ind_x = 0:x_len / abs(x_len):(x_len);
                ind_y = ind_x * (end_insdex(1) - start_insdex(1)) / (x_len);
                ind_y_up = ceil(ind_y);
                ind_y_fo = floor(ind_y);
                ind_up = ind_s + ind_x * this.Y_num +ind_y_up;
                ind_fo = ind_s + ind_x * this.Y_num +ind_y_fo;
                ground_h = 0.5 * (this.Z(ind_up) + this.Z(ind_fo));
            else
                y_len = end_insdex(1) - start_insdex(1);
                ind = start_insdex(1):(y_len / abs(y_len)):end_insdex(1);
                ground_h = this.Z(ind, start_insdex(2));
            end

        end

        function [target_loc, delta_dist, flag, cost] = checkPath_v2(this, from, to)
            %检查两点连线是否与地形碰撞
            flag = true;
            delta_dist = norm(from(1:2) - to(1:2));
            start_insdex = zeros(1, 2);
            end_insdex = zeros(1, 2);

            start_insdex(1:2) = round(from(1:2));
            end_insdex(1:2) = round(to(1:2));

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
                cost = 0;
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

            x_mid = 0.5 * (this.x_ind(1) + this.x_ind(num));
            y_mid = 0.5 * (this.y_up(1) + this.y_up(num));
            y_ind_2 = round(this.x_ind - x_mid + y_mid);
            x_ind_2 = round(this.y_up - y_mid + x_mid);
            h_2 = zeros(num, 1);
            x_ind_2(x_ind_2 > x_max) = x_max;
            x_ind_2(x_ind_2 < 1) = 1;
            y_ind_2(y_ind_2 < 1) = 1;
            y_ind_2(y_ind_2 > y_max) = y_max;

            for i = 1:1:num
                this.h_up(i, 1) = new_Height(this.x_ind(i), this.y_up(i));
                this.h_down(i, 1) = new_Height(this.x_ind(i), this.y_down(i));
                h_2(i, 1) = new_Height(x_ind_2(i), y_ind_2(i));
            end

            cost = 6 * sum(abs(diff(h_2)));
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

        function obj = get_instance(release)
            persistent ins;

            if nargin == 1 && release == false
                ins = [];
            else

                if isempty(ins) || ~isvalid(ins)

                    ins = map();

                end

            end

            obj = ins;

        end

    end

end
