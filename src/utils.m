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

        function new = new_path(Dir, patt)

            if nargin == 2
                files = dir(Dir);
                ids = zeros(length(files), 1);

                for i = 3:length(files)
                    c = strsplit(files(i).name, '_');

                    if strcmp(patt, c{1})
                        id = strsplit(c{2}, '.');
                        tmp = str2double(id{1});

                        if ~ISNAN(tmp)
                            ids(i) = tmp;
                        end

                    end

                end

                id = max(ids) + 1;

                if id == 1
                    new = [Dir, patt, '/'];
                else
                    new = [Dir, patt, '_', num2str(id), '/'];
                end

            end

        end

    end

end
