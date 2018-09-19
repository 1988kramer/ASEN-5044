function [ dy] = nonLinear(t, x_in)
G = 6.673e-11;
g = 50;
M = 5.98e24;
R = 6.37e6;
K=1000;
alpha = (G*M)/(K*R^2);
delta_u = 300 * abs(cos(t));
u_nom = G*M*x_in(3) / (K*R^2);
u = u_nom + delta_u;

dy(1) = x_in(2);
dy(2) = ((K*u - g*x_in(2)) / x_in(3)) - (G*M / (R+x_in(1))^2);
dy(3) = -u;

dy = dy';
end

