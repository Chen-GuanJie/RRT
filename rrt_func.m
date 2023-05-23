function varargout = rrt_func(varargin)
    persistent problem;
    addpath(genpath(pwd));
    args = varargin;

    if nargin == 0
        args = {'run'};
    end

    if ~isa(problem, 'rrt')
        problem = rrt_plot();
    end

    if utils.in_cell(args, 'init')
        utils.get_instance().init();
        problem.init();
    end

    if utils.in_cell(args, 'debug')
        ind = utils.in_cell(args, 'debug');

        if length(args) > ind
            eval(['problem.', args{ind + 1}, ';']);
        end

        return
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

    if utils.in_cell(args, 'show tree')
        utils.get_instance().locate_figure('search tree')
        problem.show_map(5, false);
        problem.show_search_tree();
        return
    end

end
