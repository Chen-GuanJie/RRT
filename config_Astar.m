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
conf.start = [275 122 500]; %起始点
conf.goal = [83 378 500]; %终点
conf.height_limit = 500;
conf.rate = 60;
conf.dimension = 2;
conf.height_limit=500;
