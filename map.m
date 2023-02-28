classdef map < handle

    properties (SetAccess = public)
        X
        Y
        Z
        X_num
        Y_num
        Z_num
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
            this.Z = Height;

        end

        function index = find_closest(this, x, axis)

            if axis == 0
                a = abs(this.X - x);
            else
                a = abs(this.Y - x);
            end

            [~, index] = min(a);
        end

        function flag = checkPath(this, start, endp)
            flag = true;
            start_insdex = [this.find_closest(start(1), 0), this.find_closest(start(2), 1)];
            end_insdex = [this.find_closest(endp(1), 0), this.find_closest(endp(2), 1)];
            Xn = this.X_num;
            Yn = this.Y_num;

            if end_insdex(1, 1) ~= start_insdex (1, 1)
                k_index = (end_insdex(1, 2) - start_insdex(1, 2)) / (end_insdex(1, 1) - start_insdex(1, 1));
            else
                k_index = 999999999;
            end

            if abs(k_index) > 1
                % x_max = Yn;
                y_max = Xn;
                new_Height = this.Z';
                xx = start_insdex(1, 1);
                yy = start_insdex(1, 2);
                start_insdex(1, 1) = yy;
                start_insdex(1, 2) = xx;
                xx = end_insdex(1, 1);
                yy = end_insdex(1, 2);
                end_insdex(1, 1) = yy;
                end_insdex(1, 2) = xx;
                k_index = 1 / k_index;
            else
                % x_max = Xn;
                y_max = Yn;
                new_Height = this.Z;
            end

            deltaX = end_insdex(1, 1) - start_insdex(1, 1);

            if deltaX < 0
                increase = -1;
            elseif deltaX > 0
                increase = 1;
            else
                return
            end

            for i = 0:increase:(deltaX)
                y_up = ceil((i) * k_index + start_insdex(1, 2));
                y_down = floor((i) * k_index + start_insdex(1, 2));
                h = start(1, 3) + (endp(1, 3) - start(1, 3)) * i / (deltaX);

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
            meshz(this.X, this.Y, this.Z'); hold on

        end

    end

    methods (Static)

        function x_index = get_uniqu_index(x)
            x_diff = diff(x);
            x_index = find(x_diff ~= 0);
        end

        function [g, num] = grid(x)
            x_unique = unique(x);
            a = size(x_unique);
            num = a(1, 1);
            g = sortrows(x_unique)';
            g=g-min(g);
        end

    end

end
