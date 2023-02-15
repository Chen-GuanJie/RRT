function [x_data, y_data, z_data] = SquareMap(filename)
    load(filename, 'dem_data')
    dem_data = sortrows(dem_data);
    x = dem_data(:, 1);
    y = dem_data(:, 2);
    z = dem_data(:, 3);
    [X, Xn] = grid(x);
    [Y, Yn] = grid(y);
    x_index = get_uniqu_index(x);
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

    x_data = X;
    y_data = Y;
    z_data = Height';
end

function x_index = get_uniqu_index(x)
    x_diff = diff(x);
    x_index = find(x_diff ~= 0);
end

function [g, num] = grid(x)
    x_unique = unique(x);
    a = size(x_unique);
    num = a(1, 1);
    g = sortrows(x_unique)';
end
