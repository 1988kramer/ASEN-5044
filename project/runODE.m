close all;
options = odeset('RelTol',1e-6,'AbsTol',1e-6);

[t,x] = ode45(@motionEqs, (0:0.1:40), [10 0 pi/2 -60 0 -pi/2]);

for i = 1:size(x,1)
    while x(i,3) > pi
        x(i,3) = x(i,3) - 2*pi;
    end
    while x(i,3) < -pi
        x(i,3) = x(i,3) + 2*pi;
    end
    while x(i,6) > pi
        x(i,6) = x(i,6) - 2*pi;
    end
    while x(i,6) < -pi
        x(i,6) = x(i,6) + 2*pi;
    end
end

y = [];
for i = 1:size(x,1)
    next_y = [atan((x(i,5)-x(i,2))/(x(i,4)-x(i,1)))-x(i,3);
              sqrt((x(i,1)-x(i,4))^2+(x(i,2)-x(i,5))^2);
              atan((x(i,2)-x(i,5))/(x(i,1)-x(i,5)))-x(i,6);
              x(i,4);
              x(i,5)];
    y = [y next_y];
end

figure;
subplot(3,2,1);
plot(t,x(:,1));
xlabel('timestep');
ylabel('UGV easting (m)');
subplot(3,2,2);
plot(t,x(:,4));
xlabel('timestep');
ylabel('UAV easting (m)');
subplot(3,2,3);
plot(t,x(:,2));
xlabel('timestep');
ylabel('UGV northing (m)');
subplot(3,2,4);
plot(t,x(:,5));
xlabel('timestep');
ylabel('UAV northing (m)');
subplot(3,2,5);
plot(t,x(:,3));
xlabel('timestep');
ylabel('UGV heading (rad)');
subplot(3,2,6);
plot(t,x(:,6));
xlabel('timestep');
ylabel('UAV heading (rad)');

figure;
subplot(3,2,1);
plot(t, y(1,:));
xlabel('time (s)');
ylabel('UGV - UAV bearing');
subplot(3,2,2);
plot(t, y(2,:));
xlabel('time (s)');
ylabel('UGV - UAV range');
subplot(3,2,3);
plot(t, y(3,:));
xlabel('time (s)');
ylabel('UAV - UGV bearing');
subplot(3,2,4);
plot(t, y(1,:));
xlabel('time (s)');
ylabel('UAV easting');
subplot(3,2,5);
plot(t, y(5,:));
xlabel('time (s)');
ylabel('UAV northing');
