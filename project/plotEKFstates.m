[P,x,S,e_y,true_x,y] = EKF();
t = 0.1:0.1:100;
x = x(:,2:end);
true_x = true_x(:,2:end);
ex = true_x - x;
ex(3) = constrainAngle(ex(3));
ex(6) = constrainAngle(ex(6));

% get variance values for each state variable
% at each timestep
p = zeros(6,size(P,2)/6);

for i = 2:size(x,2)
    start_index = (i-2)*6;
    for j = 1:6
        p(j,i-1) = 2*sqrt(P(j,start_index + j));
    end
end

y = y(:,2:end);

% plot state estimation errors with 2-sigma bounds
figure
subplot(3,2,1);
plot(t,ex(1,:),'b-');
hold on
plot(t,p(1,:),'b--');
plot(t,-p(1,:),'b--');
xlabel('time (s)');
ylabel('UGV easting error (m)');
legend('estimation error','2\sigma');

subplot(3,2,2);
plot(t,ex(4,:),'b-');
hold on
plot(t,p(4,:),'b--');
plot(t,-p(4,:),'b--');
xlabel('time (s)');
ylabel('UAV easting error (m)');

subplot(3,2,3);
plot(t,ex(2,:),'b-');
hold on
plot(t,p(2,:),'b--');
plot(t,-p(2,:),'b--');
xlabel('time (s)');
ylabel('UGV northing error (m)');

subplot(3,2,4);
plot(t,ex(5,:),'b-');
hold on
plot(t,p(5,:),'b--');
plot(t,-p(5,:),'b--');
xlabel('time (s)');
ylabel('UAV northing error (m)');

subplot(3,2,5);
plot(t,ex(3,:),'b-');
hold on
plot(t,p(3,:),'b--');
plot(t,-p(3,:),'b--');
xlabel('time (s)');
ylabel('UGV bearing error (rad)');

subplot(3,2,6);
plot(t,ex(6,:),'b-');
hold on
plot(t,p(6,:),'b--');
plot(t,-p(6,:),'b--');
xlabel('time (s)');
ylabel('UAV bearing error (rad)');

% plot measurements
figure
subplot(3,2,1);
plot(t,y(1,:),'b-');
xlabel('time (s)');
ylabel('\gamma_{ag} (rad)');

subplot(3,2,2);
plot(t,y(2,:),'b-');
xlabel('times (s)');
ylabel('\rho_{ga}');

subplot(3,2,3);
plot(t,y(3,:),'b-');
xlabel('times (s)');
ylabel('\gamma_{ga}');

subplot(3,2,4);
plot(t,y(4,:),'b-');
xlabel('times (s)');
ylabel('\xi_{a}');

subplot(3,2,5);
plot(t,y(5,:),'b-');
xlabel('times (s)');
ylabel('\eta_{a}');


% plot each state over time with 2-sigma bounds
figure
subplot(3,1,1);
plot(t,x(1,:),'b-');
hold on
plot(t,true_x(1,:),'r-');
plot(t,true_x(1,:)+p(1,:),'b--');
plot(t,true_x(1,:)-p(1,:),'b--');
xlabel('times (s)');
ylabel('\gamma_{ag}');
legend('estimated state','groundtruth','2 \sigma');

subplot(3,1,2);
plot(t,x(2,:),'b-');
hold on
plot(t,true_x(2,:),'r-');
plot(t,true_x(2,:)+p(2,:),'b--');
plot(t,true_x(2,:)-p(2,:),'b--');
xlabel('times (s)');
ylabel('UGV northing (m)');

subplot(3,1,3);
plot(t,x(3,:),'b-');
hold on
plot(t,true_x(3,:),'r-');
plot(t,true_x(3,:)+p(3,:),'b--');
plot(t,true_x(3,:)-p(3,:),'b--');
xlabel('times (s)');
ylabel('UGV bearing (m)');

figure
subplot(3,1,1);
plot(t,x(4,:),'b-');
hold on
plot(t,true_x(4,:),'r-');
plot(t,true_x(4,:)+p(4,:),'b--');
plot(t,true_x(4,:)-p(4,:),'b--');
xlabel('times (s)');
ylabel('UAV easting (m)');
legend('estimated state','groundtruth','2 \sigma');

subplot(3,1,2);
plot(t,x(5,:),'b-');
hold on
plot(t,true_x(5,:),'r-');
plot(t,true_x(5,:)+p(5,:),'b--');
plot(t,true_x(5,:)-p(5,:),'b--');
xlabel('times (s)');
ylabel('UAV northing (m)');

subplot(3,1,3);
plot(t,x(6,:),'b-');
hold on
plot(t,true_x(6,:),'r-');
plot(t,true_x(6,:)+p(6,:),'b--');
plot(t,true_x(6,:)-p(6,:),'b--');
xlabel('times (s)');
ylabel('UAV bearing (m)');
