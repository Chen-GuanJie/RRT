function [r, output] = config(r, ifdispaly, total_time, delay_time)

    if nargin < 4
        ifdispaly = false;
        total_time = 7;
        delay_time = 0.01;
    end

    % conf.filename = 'data/dem';
    map_name='data/Output_500.mat';
    [~,~,type_name]=fileparts(map_name);
    if strcmp(type_name,'.csv')==1
    dem_data = coder.load(map_name);
    elseif strcmp(type_name,'.mat')==1
            dem_data = coder.load(map_name);
            dem_data=dem_data.dem_data;
    end

    conf.dem_data = dem_data;
    conf.start = [324 99 500 pi * 45/180 0 0];
    conf.goal = [58 376 500 pi * 45/180 0 0];
    conf.threshold_close = 300;
    conf.threshold_goal = 6000;
    conf.search = 1.6;
    conf.randnum = [0.6 2];
    conf.max_nodes = 3000;
    conf.height_cost_rate = 15;

    conf.height_limit = 500;
    conf.deltaT = 200; %s
    conf.g = 9.8;
    conf.v = 5; %m/s
    conf.acc = 999999999;
    conf.GammaMax = 30/180 * pi;
    conf.GammaMin = -30/180 * pi;
    conf.pitchMax = 30/180 * pi;
    conf.pitchMin = -30/180 * pi;

    conf.GammaStep = 15/180 * pi; %滚转角最大步长
    conf.pitchstep = 10/180 * pi; %俯仰角最大步长
    % stable_distance = 'data/mini_stable_distance';
    % load(stable_distance, 'mini_stable_distance')
    mini_stable_distance = coder.load('data/mini_stable_distance.csv');
    conf.mini_stable_distance = mini_stable_distance;
    conf.mini_stable_distance = mini_stable_distance;
    conf.speeds = [240 260 280 300 320 340 360 380 400 440 480 520 560 600 640];

    if exist('r', 'var') && isa(r, 'rrt')
        r.set_params(conf);
    else
        clc; close all;
        r = rrt_plot(conf);

    end

    if ifdispaly
        output = r.start_star_plot(total_time, delay_time);
    else
        output = r.start_star(total_time);
    end

    % %列名称
    % col={ 'x', 'y', 'z'};
    % %生成表格，按列生成
    % result_table=table(out(:,1),out(:,2),out(:,3),'VariableNames',col);
    % %保存表格
    % writetable(result_table, 'output/path.csv');
end
