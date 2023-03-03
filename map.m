classdef map < handle

    properties (SetAccess = public)
        X
        Y
        Z
        ZT %地图转置
        X_num
        Y_num
        Z_num
        %temp value
        tmp_h
    end

    methods (Access = public)

        function this = map(arg)

            %             if isa(arg, 'str') && exist(arg, 'file')
            load(arg, 'dem_data')
            dem_data = sortrows(dem_data);
            %             elseif isa(arg, 'double')
            %                 dem_data = arg;
            %             end

            x = dem_data(:, 1);
            y = dem_data(:, 2);
            z = dem_data(:, 3);
            x = x - min(x);
            y = y - min(y);

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
                    else
                        a = find(Y == y(start));
                        b = find(Y == y(stop));

                        if (b - a) == stop - start
                            Height(i, a:b) = z((start:stop));
                        else
                            m = 1;

                            for k = 1:stop - start + 1

                                while Y(m) ~= y(k)
                                    m = m + 1;
                                end

                                Height(i, m) = z(start + k - 1);
                            end

                        end

                    end

                end

            end

            this.X = X;
            this.Y = Y;
            this.Z = Height / (X(2) - X(1));
            this.ZT = this.Z';
            this.tmp_h=zeros(1,10);
        end

        function index = find_closest(this, x, axis)

            if axis == 0
                a = abs(this.X - x);
            else
                a = abs(this.Y - x);
            end

            [~, index] = min(a);
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
                k_index = 999999999;
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
            increase = 1;

            if deltaX < 0
                increase = -1;
            elseif deltaX == 0
                return
            end

            for i = 0:increase:(deltaX / abs(deltaX))
                y_up = ceil((i) * k_index + start_insdex(1, 2));
                y_down = floor((i) * k_index + start_insdex(1, 2));
                h = start_insdex(1, 3) + (end_insdex(1, 3) - end_insdex(1, 3)) * i / (deltaX);

                if y_up > y_max
                    y_up = y_max;
                end

                if y_down < 1
                    y_down = 1;
                end

                if new_Height(start_insdex(1, 1) + i, y_up) > h
                    flag = false; break;
                end

                if new_Height(start_insdex(1, 1) + i, y_down) > h
                    flag = false; break;
                end

            end

        end

        function display_map(this)
            meshz(1:this.X_num, 1:this.Y_num, this.Z'); hold on
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
