true_x = runODE(0.0:0.1:100);
true_x = true_x';

num_runs = 10;
num_steps = 1001;

eps_x = zeros(1,num_steps-1);
eps_y = zeros(1,num_steps-1);

for i = 1:num_runs
    [P,x,S,e_y,tvec] = EKF();
    Pdim = size(P,1);
    Sdim = size(S,1);
    e_x = true_x - x;
    e_x = e_x(:,2:end);
    for j = 1:size(tvec,2)-1
        Pstart = (j-1)*Pdim + 1;
        Pend = j*Pdim;
        Sstart = (j-1)*Sdim + 1;
        Send = j*Sdim;
        eps_x(j) = eps_x(j) + (e_x(:,j)'*P(:,Pstart:Pend)*e_x(:,j));
        eps_y(j) = eps_y(j) + (e_y(:,j)'*S(:,Sstart:Send)*e_y(:,j));
    end
end

eps_x = eps_x ./ num_runs;
eps_y = eps_y ./ num_runs;