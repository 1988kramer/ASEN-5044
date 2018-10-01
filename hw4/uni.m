function y = uni(x,a,b,w)
if x > a && x <= b
    y = w * (1/(b-a));
else
    y = 0;
end
end