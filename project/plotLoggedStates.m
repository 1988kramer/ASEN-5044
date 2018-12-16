[P,x,~,~,~,~] = EKF();
t = 0.1:0.1:100;
x = x(:,2:end);

% get variance values for each state variable
% at each timestep
p = zeros(6,size(P,2)/6);

for i = 2:size(x,2)
    start_index = (i-2)*6;
    for j = 1:6
        p(j,i-1) = 2*sqrt(P(j,start_index + j));
    end
end

% plot each state over time with 2-sigma bounds
figure
subplot(3,1,1);
plot(t,x(1,:),'b-');
hold on
plot(t,x(1,:)+p(1,:),'b--');
plot(t,x(1,:)-p(1,:),'b--');
xlabel('times (s)');
ylabel('\gamma_{ag}');
legend('estimated state','2 \sigma');

subplot(3,1,2);
plot(t,x(2,:),'b-');
hold on
plot(t,x(2,:)+p(2,:),'b--');
plot(t,x(2,:)-p(2,:),'b--');
xlabel('times (s)');
ylabel('UGV northing (m)');

subplot(3,1,3);
plot(t,x(3,:),'b-');
hold on
plot(t,x(3,:)+p(3,:),'b--');
plot(t,x(3,:)-p(3,:),'b--');
xlabel('times (s)');
ylabel('UGV bearing (m)');

figure
subplot(3,1,1);
plot(t,x(4,:),'b-');
hold on
plot(t,x(4,:)+p(4,:),'b--');
plot(t,x(4,:)-p(4,:),'b--');
xlabel('times (s)');
ylabel('UAV easting (m)');
legend('estimated state','2 \sigma');

subplot(3,1,2);
plot(t,x(5,:),'b-');
hold on
plot(t,x(5,:)+p(5,:),'b--');
plot(t,x(5,:)-p(5,:),'b--');
xlabel('times (s)');
ylabel('UAV northing (m)');

subplot(3,1,3);
plot(t,x(6,:),'b-');
hold on
plot(t,x(6,:)+p(6,:),'b--');
plot(t,x(6,:)-p(6,:),'b--');
xlabel('times (s)');
ylabel('UAV bearing (m)');