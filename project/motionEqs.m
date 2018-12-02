function [dy] = motionEqs(t, x_in)
    
    L = 0.5;
    u = [2 -pi/18 12 pi/25];
    
    dy(1,1) = u(1)*cos(x_in(3));
    dy(2,1) = u(1)*sin(x_in(3));
    dy(3,1) = (u(1)/L)*tan(u(2));
    
    dy(4,1) = u(3)*cos(x_in(6));
    dy(5,1) = u(3)*sin(x_in(6));
    dy(6,1) = u(4);
end