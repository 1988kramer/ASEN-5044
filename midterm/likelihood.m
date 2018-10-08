function l = likelihood(y)
    tm = 0.61;
    te = 0.78;
    tr = 1.40;
    l = (1-exp(-(10^y)*(te - tm)))/(1-exp(-(10^y)*(tr-tm)));
end