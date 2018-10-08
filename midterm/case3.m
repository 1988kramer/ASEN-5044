function p = case3(y)
    if (y > -3 && y < 3)
        p = abs(-log(10)*10^-y)/(10^3-10^-3);
    else
        p = 0;
    end
end