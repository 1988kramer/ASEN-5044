load('midterm2_problem3c.mat');
deltaT = 0.5;
omega_a = 0.045;
omega_b = -0.045;
H = [1 0 0 0; 0 0 1 0];
R = [75 7.5; 7.5 75];
k = size(yaugHist,2); 

A_a = [0 1 0 0; 0 0 0 -omega_a; 0 0 0 1; 0 omega_a 0 0];
F_a = expm(A_a.*deltaT);

A_b = [0 1 0 0; 0 0 0 -omega_b; 0 0 0 1; 0 omega_b 0 0];
F_b = expm(A_b.*deltaT);

R_a = R + [12.5*sin(k/10) 25.5*sin(k/10); 25.5*sin(k/10) 12.5*cos(k/10)];
R_b = [8000 500; 500 8000];
R_d = blkdiag(R_a,R_b);
H_a = H*F_a;
H_b = H*F_b;
H_d = [H_a zeros(2,4); H_a H_b];
Y = yaugHist(:,1);

for i = 2:2
    next_Ha = H*(F_a^i);
    next_Hb = H*(F_b^i);
    next_Hd = [next_Ha zeros(2,4); next_Ha next_Hb];
    H_d = [H_d; next_Hd];
    next_Ra = R + [12.5*sin(i/10) 25.5*sin(i/10); 25.5*sin(i/10) 12.5*cos(i/10)];
    R_d = blkdiag(R_d,next_Ra,R_b);
    Y = [Y; yaugHist(:,i)];
end

mu = (2*H_d'*inv(R_d)*H_d)\(2*H_d'*inv(R_d)*Y);
P = inv(H_d'*inv(R_d)*H_d);
sigmas = diag(P);

for j = i:size(yaugHist,2)
    H_d = [H*(F_a^j) zeros(2,4); H*(F_a^j) H*(F_b^j)];
    R_a = R + [12.5*sin(j/10) 25.5*sin(j/10); 25.5*sin(j/10) 12.5*cos(j/10)];
    R_d = blkdiag(R_a,R_b);
    K = P*H_d'/(H_d*P*H_d'+R_d);
    next_mu = mu(:,j-1) + K*(yaugHist(:,j)-H_d*mu(:,j-1));
    mu = [mu next_mu];
    P = (eye(8)-K*H_d)*P*(eye(8)-K*H_d)'+K*R_d*K';
    sigmas = [sigmas diag(P)];
end

sigmas = 2 .* sigmas;

figure;
subplot(2,2,1)
plot(mu(1,:));
ylabel('distance (m)');
xlabel('timestep');
title('easting position for target 1');
subplot(2,2,2)
plot(mu(2,:));
ylabel('velocity (m/s)');
xlabel('timestep');
title('easting velocity for target 1');
subplot(2,2,3)
plot(mu(3,:));
ylabel('distance (m)');
xlabel('timestep');
title('northing position for target 1');
subplot(2,2,4)
plot(mu(4,:));
ylabel('velocity (m/s)');
xlabel('timestep');
title('northing velocity for target 1');

figure;
subplot(2,2,1)
plot(mu(5,:));
ylabel('distance (m)');
xlabel('timestep');
title('easting position for target 2');
subplot(2,2,2)
plot(mu(6,:));
ylabel('velocity (m/s)');
xlabel('timestep');
title('easting velocity for target 2');
subplot(2,2,3)
plot(mu(7,:));
ylabel('distance (m)');
xlabel('timestep');
title('northing position for target 2');
subplot(2,2,4)
plot(mu(8,:));
ylabel('velocity (m/s)');
xlabel('timestep');
title('northing velocity for target 2');

figure;
subplot(2,2,1)
plot(sigmas(1,:));
ylabel('distance variance (m^2)');
xlabel('timestep');
title('easting position variance for target 1');
subplot(2,2,2)
plot(sigmas(2,:));
ylabel('velocity variance (m/s)^2');
xlabel('timestep');
title('easting velocity variance for target 1');
subplot(2,2,3)
plot(sigmas(3,:));
ylabel('distance variance (m^2)');
xlabel('timestep');
title('northing position variance for target 1');
subplot(2,2,4)
plot(sigmas(4,:));
ylabel('velocity variance (m/s)^2');
xlabel('timestep');
title('northing velocity variance for target 1');

figure;
subplot(2,2,1)
plot(sigmas(5,:));
ylabel('distance variance (m^2)');
xlabel('timestep');
title('easting position variance for target 2');
subplot(2,2,2)
plot(sigmas(6,:));
ylabel('velocity variance (m/s)^2');
xlabel('timestep');
title('easting velocity variance for target 2');
subplot(2,2,3)
plot(sigmas(7,:));
ylabel('distance variance (m^2)');
xlabel('timestep');
title('northing position variance for target 2');
subplot(2,2,4)
plot(sigmas(8,:));
ylabel('velocity variance (m/s)^2');
xlabel('timestep');
title('northing velocity variance for target 2');