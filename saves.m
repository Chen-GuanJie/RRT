function rt = saves(Dir, patt, types)
    %Dir='output';
    files = dir(Dir);
    % patt = 'path';
    num = size(files);
    num = num(1);
    ids = [];
    nofile = 1;

    for i = 3:num
        c = strsplit(files(i).name, '_');

        if strcmp(patt, c{1})
            nofile = 0;
            id = strsplit(c{2}, '.');
            ids = [ids str2double(id{1})];
        end

    end

    if nofile == 1
        id = 1;
    else
        id = max(ids) + 1;
    end

    if types == 0
        rt = [Dir, '/', patt, '_', num2str(id), '.fig'];
    elseif types == 1
        rt = [Dir, '/', patt, '_', num2str(id), '.mat'];
    end

end
