deltaT = 0.5;
omega_a = 0.045;
omega_b = -0.045;
xi_R = 100;
eta_R = 100;
xl = [-xi_R -eta_R];
xu = [xi_R eta_R];
k = 300; 
J = [1 0 0 0; 0 0 1 0];

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
sigmas_b = 2 .* sqrt(diag(abs(Pb)));

mu_rc = J*(mu_a - mu_b);
Prc = J*(Pa + Pb)*J';

Pc = mvncdf(xl,xu,mu_rc',Prc);

for i = 2:k
    next_mu_a = F_a * mu_a(:,i-1);
    next_Pa = F_a * Pa(:,4*(i-1)-3:4*(i-1)) * F_a';
    mu_a = [mu_a next_mu_a];
    Pa = [Pa next_Pa];
    sigmas_a = [sigmas_a 2 .* sqrt(diag(abs(next_Pa)))];
    
    next_mu_b = F_b * mu_b(:,i-1);
    next_Pb = F_b * Pb(:,4*(i-1)-3:4*(i-1)) * F_b';
    mu_b = [mu_b next_mu_b];
    Pb = [Pb next_Pb];
    sigmas_b = [sigmas_b 2 .* sqrt(diag(abs(next_Pb)))];
    
    next_mu_rc = J*(next_mu_a - next_mu_b);
    next_Prc = J*(next_Pa + next_Pb)*J';
    next_Prc(1,2) = 0;
    next_Prc(2,1) = 0;
    mu_rc = [mu_rc next_mu_rc];
    Prc = [Prc next_Prc];
    
    next_Pc = mvncdf(xl,xu,next_mu_rc',next_Prc);
    Pc = [Pc next_Pc];
end



