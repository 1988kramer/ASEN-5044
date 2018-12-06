function y = getMeas(x)
    d52 = x(5) - x(2);
    d41 = x(4) - x(1);
    d25 = -d52;
    d14 = -d41;
    y = [atan2(d52,d41)-x(3);
         sqrt(d14^2+d25^2);
         atan2(d25,d14)-x(6);
         x(4);
         x(5)];
end