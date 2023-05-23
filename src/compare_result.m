clc; clear; close all;
dir_path = 'output/';
x_data = 'time_stamp';
dirs = {};

plot_data = {
             'node_num';
             'num_rewire';
             'num_neighbor';
             'mini_cost'
             };

if isempty (dirs)
    d = dir(dir_path);

    for i = 3:length(d)
        dirs{end + 1} = d(i).name;
    end

end

if utils.checkdir(dir_path, false)
    data = cell(length(dirs), 1);
    display_name = cell(length(dirs), 1);

    for i = 1:length(dirs)
        data{i} = readtable([dir_path, dirs{i}, '/', 'sample_result']);
        n = strsplit(dirs{i}, 'M ');
        display_name{i} = n{2};

        for j = 1:length(plot_data)
            utils.get_instance().locate_figure(plot_data{j})
            plot(data{i}.(x_data), data{i}.(plot_data{j}), 'DisplayName', display_name{i}); hold on
            title(strrep(plot_data{j}, '_', ' '));
            legend
        end

    end

end
