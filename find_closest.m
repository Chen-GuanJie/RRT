function index = find_closest(x, list)
    a = abs(list - x);
    mini = min(a);
    index = find(a == mini);
end
