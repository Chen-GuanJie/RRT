function [r, output] = rrt_start_func(r, ifdispaly, total_time, delay_time)
%{
ifdispaly 是否画图
total_time 总运行时间
delay_time 画图间隔
%}
    if nargin < 4
        ifdispaly = false;
        total_time = 7;
        delay_time = 0.01;
    end

    addpath(genpath(pwd));

    if exist('config.m', 'file')
        run([pwd '/config.m']);
    else
        disp('ERROR: There is no configuration file!')
        return
    end

    if exist('r', 'var') && isa(r, 'rrt')
        r.set_params(conf);
    else
        clc; close all;
        r = rrt_plot(conf);

    end

    if ifdispaly
output = r.start_star_plot(total_time, delay_time);

%         if delay_time <= 0
%             output = r.start_star_1_plot(total_time);
%         else
%             output = r.start_star_plot(total_time, delay_time);
%         end

    else
        [output, ~] = r.start_star(total_time);
    end

    % %列名称
    % col={ 'x', 'y', 'z'};
    % %生成表格，按列生成
    % result_table=table(output(:,1),output(:,2),output(:,3),'VariableNames',col);
    % %保存表格
    % writetable(result_table, 'output/path.csv');
end
