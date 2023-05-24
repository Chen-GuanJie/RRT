function Astar_func(varargin)
    persistent problem;
    addpath(genpath(pwd));
    args = varargin;

    if nargin == 0
        args = {'run'};
    end

    if ~isa(problem, 'Astar')
        problem = Astar();
    end

    if utils.in_cell(args, 'run')
        close all;
        utils.get_instance().init();
        problem.init();
        problem.start();
    end

end
