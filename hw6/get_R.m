function R_i = get_R(R,n)
    R_i = [];
    for i=1:n
        next_line = [];
        for j = 1:n
            if j == i
                next_line = [next_line R];
            else
                next_line = [next_line zeros(3)];
            end
        end
        R_i = [R_i; next_line];
    end
end