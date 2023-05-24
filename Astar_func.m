function output = Astar_func(varargin)
    persistent problem;
    addpath(genpath(pwd));
    args = varargin;

    if nargin == 0
        args = {'run'};
    end

    if ~isa(problem, 'Astar')
        problem = Astar();
    end

    if utils.in_cell(args, 'clear')
        utils.clear_all();
        problem = [];
        return

    elseif utils.in_cell(args, 'run')
        close all;
        utils.get_instance().init();
        problem.init();
        output = problem.start();
    end

end
