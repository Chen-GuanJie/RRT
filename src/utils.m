classdef utils < handle

    properties (SetAccess = private)
    end

    methods (Access = private)
    end

    methods (Access = public)
    end

    methods (Static)

        function checkdir(dir)

            if ~exist(dir, 'dir')
                mkdir(dir);
                disp(['creat new dir ', dir])
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

        function map_data = load_map(input)

        end

    end

end
