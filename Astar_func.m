function [a, output] = Astar_func(a, ifdispaly)
    clc; close all;
    addpath(genpath(pwd));

    if exist('config_Astar.m', 'file')
        run([pwd '/config_Astar.m']);
    else
        disp('ERROR: There is no configuration file!')
        return
    end

    if exist('a', 'var') && isa(a, 'rrt')
        a.set_params(conf);
    else
        a = Astar(conf);
    end

    if ifdispaly
        output = a.start_Astar_plot(delay_time);
    else
        output = a.start_Astar();
    end

end
