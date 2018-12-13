function x = runODE(t)
options = odeset('RelTol',1e-6,'AbsTol',1e-6);

x = [10 0 pi/2 -60 0 (-pi/2)];
x_p = [0 1 0 0 0 0.1];
y = [];

for i = 1:size(t,2)-1
    
    [t,x_ode] = ode45(@motionEqs, (0:0.1:0.1), x(i,:),[],[2 -pi/18 12 pi/25]);
    x = [x; x_ode(end,:)];
    x(i+1,3) = constrainAngle(x(i+1,3));
    x(i+1,6) = constrainAngle(x(i+1,6));
    
    d52 = x(i+1,5)-x(i+1,2);
    d41 = x(i+1,4)-x(i+1,1);
    d25 = -d52;
    d14 = -d41;
    
    next_y = [atan2(d52,d41)-x(i+1,3);
              sqrt(d14^2+d25^2);
              atan2(d25,d14)-x(i+1,6);
              x(i+1,4);
              x(i+1,5)];
    
    next_y(1) = constrainAngle(next_y(1));
    next_y(3) = constrainAngle(next_y(3));
    
    y = [y next_y];
end

t = 0:0.1:100;

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

t = 0.1:0.1:100;

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
plot(t, y(4,:));
xlabel('time (s)');
ylabel('UAV easting');
subplot(3,2,5);
plot(t, y(5,:));
xlabel('time (s)');
ylabel('UAV northing');

