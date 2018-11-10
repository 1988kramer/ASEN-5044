deltaT = 0.5;
omega_a = 0.045;
omega_b = -0.045;
xi_R = 100;
eta_R = 100;
k = 300; 

A_a = [0 1 0 0; 0 0 0 -omega_a; 0 0 0 1; 0 omega_a 0 0];
F_a = expm(A_a.*deltaT);

A_b = [0 1 0 0; 0 0 0 -omega_b; 0 0 0 1; 0 omega_b 0 0];
F_b = expm(A_b.*deltaT);

mu_a_0 = [0; 85*cos(pi/4); 0; -85*sin(pi/4)];
Pa_0 = diag([10 4 10 4]);

mu_b_0 = [3200; 85*cos(pi/4); 3200; -85*sin(pi/4)];
Pb_0 = diag([11 3.5 11 3.5]);

mu_a = F_a * mu_a_0;
Pa = F_a * Pa_0 * F_a';
sigmas_a = 2 .* sqrt(diag(abs(Pa)));

mu_b = F_b * mu_b_0;
Pb = F_b * Pb_0 * F_b';
sigmas_b = 2 .* sqrt(diag(abs(Pb));

for i = 2:k
    next_mu_a = F * mu_a(:,i-1);
    next_Pa = F_a * Pa(:,4*(i-1)-3:4*(i-1)) * F_a';
    mu_a = [mu_a next_mu_a];
    Pa = [Pa next_Pa];
    sigmas_a = [sigmas_a 2 .* sqrt(diag(abs(next_Pa)))];
end

for i = 1:4
    figure;
    plot(mu(i,:),'b-');
    hold on
    plot(mu(i,:)+sigmas(i,:),'c--');
    plot(mu(i,:)-sigmas(i,:),'c--');
    xlabel('timestep (k)')
    ylabel(sprintf('x_%i',i));
    figure
    plot(sigmas(i,:));
    xlabel('timestep (k)');
    ylabel(sprintf('2 sigma_%i',i));
end

