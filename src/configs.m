classdef configs < handle

    properties (SetAccess = private)
        config_file
        config_dir = './data/config/'
        templates_dir = './data/templates/'
        output_dir = './output/'
        data = struct
        save_path
        changed
    end

    methods (Access = public)

        function output = load(this)
            old_data = this.data;
            this.data = yaml.loadFile(this.config_file);

            if isfield(this.data, 'name') && ~strcmp(this.data.name, old_data.name)
                this.save_path = [this.output_dir char(this.data.name) '/'];
                utils.checkdir(this.save_path);
            end

            output = this.data;

        end

        function output = iterate_save_path(this)
            output = utils.iterate_path(this.save_path);
            utils.checkdir(output);
        end

        function save(this)
            yaml.dumpFile(this.save_path, this.data)
        end

    end

    methods (Access = private)

        function this = configs(name)
            this.config_file = [this.config_dir, name, '.yaml'];
            utils.checkdir(this.config_dir);

            if exist(this.config_file, 'file')
                disp([name, ' config file exist'])
            else
                disp('copy config file')
                copyfile([this.templates_dir, name, '_template.yaml'], this.config_file)
            end

            this.data.name = '';
        end

    end

    methods (Static)

        function conf = get_config(name)
            persistent all_configs;

            if nargin > 0

                if (isempty(all_configs) || ~isvalid(all_configs))
                    all_configs = containers.Map();
                end

                if ~isKey(all_configs, name)
                    all_configs(name) = configs(name);
                end

                conf = all_configs(name);
            else
                conf = all_configs;
            end

        end

    end

end
