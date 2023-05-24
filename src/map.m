classdef map < handle

    properties (SetAccess = public)
        X = zeros(1, 1, 'single')
        Y = zeros(1, 1, 'single')
        Z = zeros(1, 1, 'single')
        X_num = 0
        Y_num = 0
        Z_num = 0
        start_point
        goal
        map_scale
        height_limit
    end

    properties (SetAccess = private)
        name = 'map'
        config_manger
        rand_id = -1
        map_path = './data/map/'
        map_name = ''
        ZT = zeros(1, 1, 'single') %地图转置
        size_map = zeros(1, 2, 'single')
        size_mapT = zeros(1, 2, 'single')
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
            [X_data, Xn] = this.grid(x);
            [Y_data, Yn] = this.grid(y);
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

            this.X = X_data;
            this.Y = Y_data;
            this.map_scale = this.X(2) - this.X(1);
            this.save_built_map(this.X, this.Y, Height);
            this.Z = Height / this.map_scale;
            this.ZT = this.Z';
        end

    end

    methods (Access = public)

        function init(this)
            this.rand_id = rand;
            conf = this.config_manger.load(this.rand_id);
            this.start_point = cell2mat(conf.start_point);
            this.goal = cell2mat(conf.goal);
            conf.map_name = char(conf.map_name);

            if ~strcmp(this.map_name, conf.map_name) || ~this.validate_map()
                this.map_name = conf.map_name;
                n = [this.map_path, this.map_name];

                if (isfile([n, '_x.mat']) || isfile([n, '_x.csv'])) && ...
                        (isfile([n, '_y.mat']) || isfile([n, '_y.csv'])) && ...
                        (isfile([n, '_z.mat']) || isfile([n, '_z.csv']))
                    disp('find built map')
                    this.X = utils.load_file(this.map_path, [this.map_name, '_x']);
                    this.map_scale = this.X(2) - this.X(1);
                    this.Y = utils.load_file(this.map_path, [this.map_name, '_y']);
                    this.Z = utils.load_file(this.map_path, [this.map_name, '_z']) / this.map_scale;
                    this.ZT = this.Z';
                    this.X_num = length(this.X);
                    this.Y_num = length(this.Y);
                    this.Z_num = size(this.Z, 1) * size(this.Z, 2);

                else
                    disp('build map')
                    this.build_map(utils.load_file(this.map_path, this.map_name));
                end

            end

            this.size_map = [this.X_num, this.Y_num];
            this.size_mapT = [this.Y_num, this.X_num];

            if this.start_point(3) == 0
                this.start_point(3) = this.Z(this.start_point(1), this.start_point(2));
            end

            if this.goal(3) == 0
                this.goal(3) = this.Z(this.goal(1), this.goal(2));
            end

            this.height_limit = conf.height_limit / this.map_scale;
        end

        function [ground_h, flag] = checkPath(this, from, to)
            flag = true;
            start_insdex = round(from(1:2));
            end_insdex = round(to(1:2));

            if all (start_insdex == end_insdex) || any(start_insdex < 1) || ...
                    any(end_insdex < 1) || start_insdex(1) > this.X_num || ...
                    end_insdex(1) > this.X_num || start_insdex(2) > this.Y_num || ...
                    end_insdex(2) > this.Y_num
                ground_h = []; flag = false;
                return
            end

            x_len = end_insdex(2) - start_insdex(2);
            y_len = end_insdex(1) - start_insdex(1);

            if abs(x_len) >= abs(y_len)
                ind_s = sub2ind(this.size_map, start_insdex(1), start_insdex(2));

                if x_len ~= 0
                    ind_x = 0:x_len / abs(x_len):(x_len);
                    ind_y = ind_x * y_len / (x_len);
                    ind_up = ind_s + ind_x * this.X_num + ceil(ind_y);
                    ind_fo = ind_s + ind_x * this.X_num + floor(ind_y);
                    ground_h = 0.5 * (this.Z(ind_up) + this.Z(ind_fo));
                else
                    ind = start_insdex(1):(y_len / abs(y_len)):end_insdex(1);
                    ground_h = this.Z(ind, start_insdex(2));
                end

            else
                ind_s = sub2ind(this.size_mapT, start_insdex(2), start_insdex(1));

                if y_len ~= 0
                    ind_y = 0:y_len / abs(y_len):(y_len);
                    ind_x = ind_y * x_len / (y_len);
                    ind_up = ind_s + ind_y * this.Y_num + ceil(ind_x);
                    ind_fo = ind_s + ind_y * this.Y_num + floor(ind_x);
                    ground_h = 0.5 * (this.ZT(ind_up) + this.ZT(ind_fo));

                else
                    ind = start_insdex(2):(x_len / abs(x_len)):end_insdex(2);
                    ground_h = this.ZT(ind, start_insdex(1));
                end

            end

        end

        function shit = to_normal_size(this, shit)
            shit = shit * this.map_scale;
            shit(:, 1) = shit(:, 1) + min(this.X);
            shit(:, 2) = shit(:, 2) + min(this.Y);
        end

        function display_map(this, interval, normal_map)

            if nargin < 3
                interval = 5;
                normal_map = true;
            end

            x = 1:interval:this.X_num;
            y = 1:interval:this.Y_num;
            z = this.ZT(y, x);
            z = z - this.height_limit;

            if normal_map
                x = this.X(x);
                y = this.Y(y);
                z = z * this.map_scale;
            end

            meshz(x, y, z);

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
