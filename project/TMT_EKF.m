
num_runs = 20;
num_steps = 401;
alpha = 0.90;
n = 6;
p = 5;

eps_x = zeros(num_runs,num_steps-1);
eps_y = zeros(num_runs,num_steps-1);

for i = 1:num_runs
    [P,x,S,e_y,true_x] = EKF();
    Pdim = size(P,1);
    Sdim = size(S,1);
    e_x = true_x - x;
    e_x = e_x(:,2:end);
    for j = 1:num_steps-1
        Pstart = (j-1)*Pdim + 1;
        Pend = j*Pdim;
        Sstart = (j-1)*Sdim + 1;
        Send = j*Sdim;
        eps_x(i,j) = (e_x(:,j)'*inv(P(:,Pstart:Pend))*e_x(:,j));
        eps_y(i,j) = (e_y(:,j)'*inv(S(:,Sstart:Send))*e_y(:,j));
    end
end

t = 0.1:0.1:40;

r1_y = chi2inv(alpha/2,num_runs*p)/num_runs;
r1_y = r1_y * ones(size(t));
r2_y = chi2inv(1-(alpha/2),num_runs*p)/num_runs;
r2_y = r2_y * ones(size(t));

r1_x = chi2inv(alpha/2,num_runs*n)/num_runs;
r1_x = r1_x * ones(size(t));
r2_x = chi2inv(1-(alpha/2),num_runs*n)/num_runs;
r2_x = r2_x * ones(size(t));

eps_x = sum(eps_x,1) ./ num_runs;
eps_y = sum(eps_y,1) ./ num_runs;

scatter(t,eps_x);
hold on;
plot(t,r1_x,'b--',t,r2_x,'b--');
figure;
scatter(t,eps_y);
hold on;
plot(t,r1_y,'b--',t,r2_y,'b--');