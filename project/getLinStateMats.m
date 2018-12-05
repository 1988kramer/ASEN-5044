function [F,G] = getLinStateMats(x,u,L,deltaT)
    A = [0 0 -u(1)*sin(x(3)) 0 0 0;
         0 0  u(1)*cos(x(3)) 0 0 0;
         0 0  0              0 0 0;
         0 0  0              0 0 -u(3)*sin(x(6));
         0 0  0              0 0  u(3)*cos(x(6));
         0 0  0              0 0  0];
    B = [cos(x(3)) 0 0 0;
         sin(x(3)) 0 0 0;
         (1/L)*tan(u(2)) (u(1)/L)*(tan(u(2))^2+1) 0 0; 
         0 0 cos(x(6)) 0;
         0 0 sin(x(6)) 0;
         0 0 0         1];
     
    F = eye(size(A)) + deltaT * A;
    G = deltaT * B;
end