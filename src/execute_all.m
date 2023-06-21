clc; clear;
config_dir = './data/config/';
templates_dir = './data/templates/';
conf = yaml.loadFile([templates_dir, 'excute_all.yaml']);
missions = fieldnames(conf);
need = {'benchmark', 'classify', 'display', 'map', 'rrt', 'uav'};
mission_path = cell(length(missions), 1);

for i = 1:length(missions)
    dir_path = [char(config_dir), char(missions{i}), '/'];
    mission_path{i} = dir_path;
    utils.checkdir(dir_path);

    for j = 1:length(need)
        config_file = [dir_path, need{j}, '.yaml'];

        % if ~exist(config_file, 'file')
        %     disp(['copy ', need{j}, ' config template'])
        copyfile([templates_dir, need{j}, '_template.yaml'], config_file)
        % end

    end

    files_change = conf.(missions{i});
    need_change = fieldnames(files_change);

    for k = 1:length(need_change)
        old = yaml.loadFile([dir_path, need_change{k}, '.yaml']);
        old = utils.assign_struct(old, files_change.(need_change{k}));
        yaml.dumpFile([dir_path, need_change{k}, '.yaml'], old)
    end

end

for i = 1:length(missions)
    t = rrt_func('config', mission_path{i}, 'init', 'debug', 'get_repeat_times', 1);
    disp(['run ', missions{i}]);
    rrt_func('run', [missions{i}])

    for j = 2:t.func.get_repeat_times
        disp(['run ', missions{i}, '-', num2str(j)]);
        rrt_func('run', [missions{i}, '-', num2str(j)])
    end

end
