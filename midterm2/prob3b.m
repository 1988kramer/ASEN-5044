load('midterm2_problem3b.mat');
deltaT = 0.5;
omega_a = 0.045;
H = [1 0 0 0; 0 0 1 0];
R = [75 7.5; 7.5 75];
k = size(yaHist,2); 

A_a = [0 1 0 0; 0 0 0 -omega_a; 0 0 0 1; 0 omega_a 0 0];
F_a = expm(A_a.*deltaT);

R_a = R + [12.5*sin(1/10) 25.5*sin(1/10); 25.5*sin(1/10) 12.5*cos(1/10)];
H_a = H*F_a;
Y = yaHist(:,1);

for i = 2:k
    R_a = [R_a; zeros(2,size(R_a,2))];
    next_R = R + [12.5*sin(i/10) 25.5*sin(i/10); 25.5*sin(i/10) 12.5*cos(i/10)];
    next_R = [zeros(size(R_a,1)-2,2); next_R];
    R_a = [R_a next_R];
    H_a = [H_a; H*(F_a.^k)];
    Y = [Y; yaHist(:,k)];
end



