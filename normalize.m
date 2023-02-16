function new_x = normalize(x)
a=min(min(x));
    new_x = x - min(min(x));
    new_x = new_x ./ max(new_x);
end
