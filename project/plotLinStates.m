x0 = [10; 0; pi/2; -60; 0; -pi/2];
u = [2; -pi/18; 12; pi/25];
x = x0;
y = [];
t = [0:0.1:40];

%[F,G,H,M] = getLinearizedMatrices(x0,u,0.5,0.1);


for i = 1:400
    [F,G,H,M] = getLinearizedMatrices(xnom(:,i),u,0.5,0.1);
    new_x = F*x(:,i) + G*u;
    new_y = H*x(:,i);
    x = [x new_x];
    y = [y new_y];
end

for i = 1:size(x,2)
    while x(3,i) > pi
        x(3,i) = x(3,i) - 2*pi;
    end
    while x(3,i) < -pi
        x(3,i) = x(3,i) + 2*pi;
    end
    while x(6,i) > pi
        x(6,i) = x(6,i) - 2*pi;
    end
    while x(6,i) < -pi
        x(6,i) = x(6,i) + 2*pi;
    end
end

figure;
subplot(3,2,1);
plot(t,x(1,:));
xlabel('time (s)');
ylabel('UGV easting (m)');
subplot(3,2,2);
plot(t,x(4,:));
xlabel('time (s)');
ylabel('UAV easting (m)');
subplot(3,2,3);
plot(t,x(2,:));
xlabel('time (s)');
ylabel('UGV northing (m)');
subplot(3,2,4);
plot(t,x(5,:));
xlabel('time (s)');
ylabel('UAV northing (m)');
subplot(3,2,5);
plot(t,x(3,:));
xlabel('time (s)');
ylabel('UGV heading (rad)');
subplot(3,2,6);
plot(t,x(6,:));
xlabel('time (s)');
ylabel('UAV heading (rad)');

figure;
subplot(3,2,1);
plot(t(2:end), y(1,:));
xlabel('time (s)');
ylabel('UGV - UAV bearing');
subplot(3,2,2);
plot(t(2:end), y(2,:));
xlabel('time (s)');
ylabel('UGV - UAV range');
subplot(3,2,3);
plot(t(2:end), y(3,:));
xlabel('time (s)');
ylabel('UAV - UGV bearing');
subplot(3,2,4);
plot(t(2:end), y(1,:));
xlabel('time (s)');
ylabel('UAV easting');
subplot(3,2,5);
plot(t(2:end), y(5,:));
xlabel('time (s)');
ylabel('UAV northing');
