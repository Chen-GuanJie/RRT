classdef utils < handle

    properties (SetAccess = private)
        figures
        used_figure_id
    end

    methods (Access = private)

        function this = utils()
            this.figures = containers.Map();
            this.used_figure_id = [];
        end

        function id = unused_id(this)

            for i = 1:length(this.used_figure_id)

                if i < this.used_figure_id(i)
                    this.used_figure_id(end + 1) = i;
                    id = i;
                    this.used_figure_id = sortrows(this.used_figure_id);
                    return;
                end

            end

            if isempty(this.used_figure_id)
                this.used_figure_id(end + 1) = 1;
                id = 1;
                return;
            else
                this.used_figure_id(end + 1) = length(this.used_figure_id) + 1;
                id = length(this.used_figure_id) + 1;
            end

            this.used_figure_id = sortrows(this.used_figure_id);
        end

    end

    methods (Access = public)

        function init(this)
            this.figures = containers.Map();
            this.used_figure_id = [];
        end

        function save_figures(this, path)
            names = keys(this.figures);

            for i = 1:this.figures.Count
                figure = this.figures(names{i});
                formats = strsplit(figure.format, ' ');

                for j = 1:length(formats)
                    saveas(figure.picture, [path, names{i}, '.', formats{j}]);
                end

            end

        end

        function is_exist = locate_figure(this, figure_name, format)
            is_exist = false;

            if nargin == 2 || (nargin == 3 && yaml.isNull(format))
                format = {'fig'};
            end

            if ~isKey(this.figures, figure_name)
                id = this.unused_id();
                this.figures(figure_name) = struct('picture', figure(id), 'id', id, 'format', format);
            elseif ~ishandle(this.figures(figure_name))
                id = this.figures(figure_name).id;
                this.figures(figure_name) = struct('picture', figure(id), 'id', id, 'format', format);
            else
                is_exist = true;
                figure(this.figures(figure_name).id);
            end

        end

        function delete_figure(this, figure_name)
            delete(this.figures(figure_name).picture);
            this.used_figure_id(this.used_figure_id == this.figures(figure_name).id) = [];
            remove(this.figures, figure_name);
        end

    end

    methods (Static)

        function is_exist = checkdir(dir, make_if_none)
            is_exist = true;

            if ~exist(dir, 'dir')

                if ~(nargin == 2 && make_if_none == false)
                    mkdir(dir);
                    disp(['creat new dir ', dir])
                end

                is_exist = false;
            end

        end

        function obj = get_instance()
            persistent ins;

            if (isempty(ins) || ~isvalid(ins))

                ins = utils();

            end

            obj = ins;

        end

        function id = id_dir(dir_name)
            c = strsplit(dir_name, '_');
            id = str2double(c{end});
        end

        function output = assert_patten(patt, name, type)
            output = false;

            if nargin == 2
                c = strsplit(name, '_');

                if strcmp(patt, '') && length(c) == 1

                    output = true;
                    return
                end

                if strcmp(patt, c{1})
                    output = true;
                end

            elseif nargin == 3
                c = strsplit(name, {'_', '.'});

                if strcmp(type, c{end}) && strcmp(patt, c{1})
                    output = true;
                end

            end

        end

        function id = id_file(file_name)
            c = strsplit(file_name, {'_', '.'});
            id = str2double(c{end - 1});
        end

        function new = iterate_path(Dir, patt, type)

            files = dir(Dir);
            ids = zeros(length(files), 1);

            switch nargin
                case 1

                    for i = 3:length(files)
                        patt = '';

                        if files(i).isdir && utils.assert_patten(patt, files(i).name)
                            ids(i) = utils.id_dir(files(i).name);
                        end

                    end

                    id = max(ids) + 1;
                    new = [Dir, num2str(id), '/'];

                case 2

                    for i = 3:length(files)

                        if files(i).isdir && utils.assert_patten(patt, files(i).name)
                            ids(i) = utils.id_dir(files(i).name);
                        end

                    end

                    id = max(ids) + 1;
                    new = [Dir, patt, '_', num2str(id), '/'];

                case 3

                    for i = 3:length(files)

                        if ~files(i).isdir && utils.assert_patten(patt, files(i).name, type)
                            ids(i) = utils.id_file(files(i).name);
                        end

                    end

                    id = max(ids) + 1;
                    new = [Dir, patt, '_', num2str(id), '.', type];

            end

        end

        function data = load_file(path, name)

            if utils.checkdir(path, false)

                if exist([path, name, '.mat'], 'file')
                    m = load([path, name, '.mat'], '-mat');
                    n = fieldnames(m);
                    data = m.(n{1});

                elseif exist([path, name, '.csv'], 'file')
                    data = load([path, name, '.csv']);
                else
                    disp([name, ' not exist'])

                end

            else
                disp([path, ' not exist'])
            end

        end

        function clear_all()
            map.get_instance(false);
            configs.get_config(false);
        end

        function output = in_cell(c, item)
            output = find(strcmp(c, item));
        end

        function flag = is_drawable(plot_name, plot_info)

            if ~isfield(plot_info, 'x_lable') || yaml.isNull(plot_info.x_lable) ...
                    || ~isfield(plot_info, 'y_lable') || yaml.isNull(plot_info.y_lable) ...
                    || ~isfield(plot_info, 'y_data') || yaml.isNull(plot_info.y_data)
                flag = false; return
            end

            flag = true;
            utils.get_instance().locate_figure(plot_name, plot_info.save_format);
            hold on
        end

        function assign_value(obj, properties)
            property = fieldnames(properties);

            for k = 1:length(property)
                obj.(property{k}) = properties.(property{k});
            end

        end

        function draw(obj, x, y_info)

        end

    end

end
