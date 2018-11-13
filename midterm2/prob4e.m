clear;
close all;

k = 500;
F = [0.99 0.2; 0 -0.76];
Q = [1 0.37; 0.37 2.5];

P_a = Q;
P_a_log = [];
P_mat = dlyap(F,Q);

for i=1:k
    P_a_log = [P_a_log [P_a(1,1);P_a(1,2);P_a(2,2)]];
    P_a = P_a + F^i*Q*(F')^i;
end

P = 10.*eye(2);
P_log = [];

for i = 1:k
    P_log = [P_log [P(1,1);P(1,2);P(2,2)]];
    P = F*P*F' + Q;
end

figure;
plot(P_a_log(1,1:500));
hold on
plot(P_mat(1,1).*(ones(1,500)));
plot(P_log(1,1:500));
xlabel('iteration');
ylabel('\sigma_{1,1}^2');
legend('analytical', 'dlyap', 'simulated');

figure;
plot(P_a_log(2,1:25));
hold on
plot(P_mat(1,2).*(ones(1,25)));
plot(P_log(2,1:25));
xlabel('iteration');
ylabel('\sigma_{1,2}^2');
legend('analytical', 'dlyap', 'simulated');

figure;
plot(P_a_log(3,1:25));
hold on
plot(P_mat(2,2).*(ones(1,25)));
plot(P_log(3,1:25));
xlabel('iteration');
ylabel('\sigma_{2,2}^2');
legend('analytical', 'dlyap', 'simulated');

