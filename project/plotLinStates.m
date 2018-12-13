x0 = [10; 0; pi/2; -60; 0; -pi/2];
x_tilde = [0;1;0;0;0;0.1];
u = [2; -pi/18; 12; pi/25];
x = x0+x_tilde;
y = [];
t = [0:0.1:100];


for i = 1:1000
    
    [F,G] = getLinStateMats(xnom(:,i+1),u,0.5,0.1);
    H = getLinHMat(xnom(:,i+1));
    
    new_x_tilde = F*x_tilde(:,i);
    x_tilde(3) = constrainAngle(x_tilde(3));
    x_tilde(6) = constrainAngle(x_tilde(6));
    x_tilde = [x_tilde new_x_tilde];
    
    new_x = xnom(:,i+1) + new_x_tilde;
    x = [x new_x];
    
    y_pret = H*x_tilde(:,i);
    y_nom = getMeas(xnom(:,i+1));
    new_y = y_nom + y_pret;
    new_y(1) = constrainAngle(new_y(1));
    new_y(3) = constrainAngle(new_y(3));

    y = [y new_y];
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
plot(t(2:end), y(4,:));
xlabel('time (s)');
ylabel('UAV easting');
subplot(3,2,5);
plot(t(2:end), y(5,:));
xlabel('time (s)');
ylabel('UAV northing');

figure;
subplot(3,2,1);
plot(t,x_tilde(1,:));
xlabel('time (s)');
ylabel('UGV easting perturbation');
subplot(3,2,2);
plot(t,x_tilde(2,:));
xlabel('time (s)');
ylabel('UGV northing perturbation');
subplot(3,2,3);
plot(t,x_tilde(3,:));
xlabel('time (s)');
ylabel('UGV bearing perturbation');
subplot(3,2,4);
plot(t,x_tilde(4,:));
xlabel('time (s)');
ylabel('UAV easting perturbation');
subplot(3,2,5);
plot(t,x_tilde(5,:));
xlabel('time (s)');
ylabel('UAV northing perturbation');
subplot(3,2,6);
plot(t,x_tilde(6,:));
xlabel('time (s)');
ylabel('UAV bearing perturbation');