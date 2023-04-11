if ~exist('dem_data', 'var')

    map_name = 'data/Output_500.mat'; %地图文件名
    [~, ~, type_name] = fileparts(map_name);

    if strcmp(type_name, '.csv') == 1
        dem_data = coder.load(map_name);
    elseif strcmp(type_name, '.mat') == 1
        dem_data = coder.load(map_name);
        dem_data = dem_data.dem_data; %mat文件中的变量名设置为dem_data
    end

end

conf.dem_data = dem_data;
%todo: 将经纬度转换为下标
conf.start = [324 99 500 pi * 45/180 0 0]; %起始点
conf.goal = [58 376 500 pi * 45/180 0 0]; %终点
conf.threshold_close = 300;
conf.threshold_goal = 6000;
conf.search = 1.6;
conf.randnum = [0.6 2];
conf.max_nodes = 3000;
conf.height_cost_rate = 12;
conf.map_scale = 500;
conf.direct_step = 8;

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
mini_stable_distance = coder.load('data/mini_stable_distance.csv');
conf.mini_stable_distance = mini_stable_distance;
conf.speeds = [240 260 280 300 320 340 360 380 400 440 480 520 560 600 640];
