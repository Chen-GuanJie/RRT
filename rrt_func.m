function output = rrt_func(varargin)
    persistent problem;
    % persistent class_info;
    output = struct;
    addpath(genpath(pwd));
    args = varargin;

    if nargin == 0
        args = {'run'};
    end

    if ~isa(problem, 'rrt')
        problem = rrt_plot();
        % class_info = ?rrt_plot;
    end

    if utils.in_cell(args, 'config')
        ind = utils.in_cell(args, 'config');

        if length(args) > ind
            problem.get_new_config(args{ind + 1});
        end

    end

    if utils.in_cell(args, 'init')
        utils.get_instance().init();
        problem.init();
    end

    if utils.in_cell(args, 'clear')
        utils.clear_all();
        problem = [];
        return
    elseif utils.in_cell(args, 'delete tree')

        problem.delete_search_tree();
        return
    elseif utils.in_cell(args, 'run')
        close all;
        utils.get_instance().init();
        problem.init();
        ind = utils.in_cell(args, 'real time');

        if ind

            if (length(args) > ind && isnumeric(args{ind + 1}))
                delay_time = args{ind + 1};
            else
                delay_time = 0.01;
            end

            problem.start_rtdisplay(delay_time);
        else
            problem.start();
        end

        problem.reshape_states();
        problem.show_result();
        ind = utils.in_cell(args, 'run');

        if length(args) > ind && ischar(args{ind + 1})
            problem.save_all(args{ind + 1});
        else
            problem.save_all();
        end

    end

    output = utils.assign_struct(output, get(problem, args));
    output = utils.assign_struct(output, debug_func(problem, args));

    if utils.in_cell(args, 'show tree')
        utils.get_instance().locate_figure('search tree')
        problem.show_map(5, false);
        problem.show_search_tree();
        return
    end

end

function output = get(obj, args)
    output.prop = struct;
    ind_get = utils.in_cell(args, 'get');

    if length(ind_get) == 1

        % if utils.check_class_prop(class_info, 'prop', args{ind_get(1) + 1});
        output.prop.(args{ind_get(1) + 1}) = obj.(args{ind_get(1) + 1});
        % end

    elseif length(ind_get) > 1
        % ind = utils.check_class_prop(class_info, 'prop', args{ind_get(1) +1:ind_get(2) - 1});

        for i = 1:ind_get(2) - ind_get(1) - 1
            output.prop.(args{ind_get(1) +i}) = obj.(args{ind_get(1) + i});
        end

    end

end

function output = debug_func(obj, args)
    output.func = struct;
    ind_debug = utils.in_cell(args, 'debug');

    if length(ind_debug) == 1

        try
            func_name = split(args{ind_debug + 1}, '(');
            output.func.(func_name{1}) = eval(['obj.', args{ind_debug + 1}, ';']);
        catch
            eval(['obj.', args{ind_debug + 1}, ';']);
        end

    elseif length(ind_debug) > 1

        for i = 1:ind_debug(2) - ind_debug(1) - 1

            try
                func_name = split(args{ind_debug(1) +i}, '(');
                output.prop.(func_name{1}) = eval(['obj.', args{ind_debug + 1}, ';']);
            catch
                eval(['obj.', args{ind_debug + 1}, ';']);
            end

        end

    end

end
