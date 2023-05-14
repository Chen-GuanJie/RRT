function rrt_func(varargin)
    persistent problem;
    addpath(genpath(pwd));
    args = varargin;

    if nargin == 0
        args = {'run'};
    end

    if ~isa(problem, 'rrt')
        problem = rrt_plot();
    end

    if strcmp(args{1}, 'debug')

        if length(args) > 1
            eval(['problem.', args{2}, ';']);
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

        problem.set_params();

        if utils.in_cell(args, 'real_time')
            problem.start_star_rtdisplay();
        else
            problem.start();
        end

        problem.reshape_states();
        problem.show_result();
        problem.save_all();
    end

    if utils.in_cell(args, 'show tree')
        problem.show_map();
        problem.show_search_tree();
        return
    end

end
