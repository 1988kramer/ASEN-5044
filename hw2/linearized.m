function [ dy] = linearized(t, x_in)
G = 6.673e-11;
g = 50;
M = 5.98e24;
R = 6.37e6;
K=1000;
alpha = (G*M)/(K*R^2);
delta_u = 10 * abs(cos(t));
u_nom = G*M*x_in(3) / (K*R^2);%alpha * 1000 * exp(-alpha * t);
u = u_nom + delta_u;

dy(1) = x_in(2);
dy(2) = (2*G*M/R^3)*x_in(1) - (g/(1000*exp(-alpha*t)))*x_in(2) + (K/(1000*exp(-alpha*t)))*(u - alpha*x_in(3));
dy(3) = -u;

dy = dy';
end

