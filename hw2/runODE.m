close all;
options = odeset('RelTol',1e-6,'AbsTol',1e-6);

%t=0->5
[t_1, x] = ode45(@nonLinear, [0 5], [0 0 1000], options);
[t_2, delta_x] = ode45(@linearized, [0 5], [0 0 1000], options);
figure;
hold on;
plot(t_1, x(:,1), 'b');
plot(t_2, delta_x(:,1), 'r');
%legend('nonlinear', 'linearized'); %, 'velocity', 'mass');
%title('Plot of Rocket Altitude');
%{
figure;
plot(t_1, x(:,2), 'b');
plot(t_2, delta_x(:,2), 'r');
figure;
plot(t_1, x(:,3), 'b');
plot(t_2, delta_x(:,3), 'r');
%}