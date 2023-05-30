clc; clear;
utils.get_instance().init();
compare_config_path = './data/config/compare/';
config_files = dir(compare_config_path);

for i = 3:length(config_files)
    compare([compare_config_path, config_files(i).name]);
end

function compare(config_file)
    close all;
    conf = yaml.loadFile(config_file);
    compare_items = conf.compare;
    display = conf.display;
    plots = fieldnames(display);
    data = cell(length(compare_items), 1);
    % display_name = cell(length(compare_items), 1);
    prop = cell(length(compare_items), 1);

    files = dir(conf.path);
    mission_map2_path = containers.Map();

    for i = 3:length(files)

        try
            n = strsplit(files(i).name, 'M ');
            mission_map2_path(n{2}) = files(i).name;
        catch
            continue
        end

    end

    for i = 1:length(compare_items)
        file_name = [conf.path, mission_map2_path(compare_items{i}.name), '/', 'sample_result'];
        file_name = join(file_name, '');
        data{i} = table2struct(readtable(file_name), 'ToScalar', true);
        %     display_name{i} = compare_items{i}.name;
        prop{i} = rmfield (compare_items{i}, 'name');
    end

    for i = 1:length(plots)
        plot_name = plots{i};
        plot_info = display.(plot_name);

        if ~utils.is_drawable(plot_name, plot_info)
            continue
        end

        % t = title(strrep(plot_name, '_', ' '));
        xl = xlabel(plot_info.x_lable.txt);
        yl = ylabel(plot_info.y_lable.txt);
        % utils.assign_value(t, plot_info, 'title_property');
        utils.assign_value(xl, plot_info.x_lable, 'property');
        utils.assign_value(yl, plot_info.y_lable, 'property');

        for j = 1:length(compare_items)
            y_name = fieldnames(plot_info.y_data);
            %         plot_info.y_data.(y_name{1}).DisplayName = display_name{j};
            fn = fieldnames(prop{j});

            for k = 1:length(fn)
                plot_info.y_data.(y_name{1}).(fn{k}) = prop{j}.(fn{k});
            end

            utils.draw(data{j}, plot_info);
        end

        if isfield(plot_info, 'eval')
            funcs = fieldnames(plot_info.eval);

            for index = 1:length(funcs)
                utils.eval_execute(funcs{index}, plot_info.eval.(funcs{index}));
            end

        end

    end

    utils.checkdir(conf.save_path, true);
    utils.get_instance().save_figures(char(conf.save_path));

end
