function new_x = normalize(x)
    new_x = x - min(x);
    new_x = new_x ./ max(new_x);
end
