function [F,G,H,M] = getLinearizedMatrices(x,u,L,deltaT)
    A = [0 0 -u(1)*sin(x(3)) 0 0 0;
         0 0 -u(1)*cos(x(3)) 0 0 0;
         0 0  0              0 0 0;
         0 0  0              0 0 -u(3)*sin(x(6));
         0 0  0              0 0  u(3)*cos(x(6));
         0 0  0              0 0  0];
    B = [cos(x(3)) 0 0 0;
         sin(x(3)) 0 0 0;
         (1/L)*tan(u(2)) (u(1)/L)*sec(u(2))^2 0 0; 
         0 0 cos(x(6)) 0;
         0 0 sin(x(6)) 0;
         0 0 0         1];
         
    d14 = x(1) - x(4);
    d41 = -d14;
    d25 = x(2) - x(5);
    d52 = -d25;
    C = [d52/(d41^2+d52^2) -1/(d41+(d52^2/d41)) -1 -d52/(d41^2+d52^2) 1/(d41+(d52^2/d41)) 0;
         d14/sqrt(d14^2+d25^2) d52/sqrt(d14^2+d25^2) 0 d41/sqrt(d14^2+d25^2) d52/sqrt(d14^2+d25^2) 0;
         -d25/(d14^2+d25^2) 1/(d14+(d25^2/d14)) 0 d25/(d14^2+d25^2) -1/(d14+(d25^2/d14)) -1;
         0 0 0 1 0 0;
         0 0 0 0 1 0];
    D = [0 0 0 0;
         0 0 0 0;
         0 0 0 0;
         0 0 0 0;
         0 0 0 0];
    Ahat = [A B;
            zeros(size(B,2),size(A,2)+size(B,2))];
    eAt = expm(Ahat*deltaT);
    F = eAt(1:size(A,1),1:size(A,2));
    G = eAt(1:size(B,1),end-size(B,2):end);
    H = C;
    M = D;
end