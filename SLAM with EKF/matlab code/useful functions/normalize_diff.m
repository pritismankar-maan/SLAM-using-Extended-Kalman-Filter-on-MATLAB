function nor_diff = normalize_diff(diff)
    for i=2:2:length(diff)
        diff(i) = normalize_angle(diff(i));
    end
nor_diff = diff;
end