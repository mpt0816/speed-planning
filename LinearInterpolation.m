function out = LinearInterpolation(key, key0, value0, key1, value1)

if key < key0 - 1e-5
    out = value0;
    return;
end

if key > key1 + 1e-5
    out = value1;
    return;
end

ratio = (key - key0) / (key1 - key0);
out = value0 + ratio * (value1 - value0);

end

