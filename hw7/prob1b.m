deltaT = 0.5;
omega = 0.045;
k = 150; 

A = [0 1 0 0; 0 0 0 -omega; 0 0 0 1; 0 omega 0 0];
F = expm(A.*deltaT);

mu_0 = [0; 85*cos(pi/4); 0; -85*sin(pi/4)];
P_0 = diag([10 2 10 2]);

mu = F * mu_0;
P = F * P_0 * F';
sigmas = 2 .* sqrt(diag(abs(P)));

for i = 2:k
    next_mu = F * mu(:,i-1);
    next_P = F * P(:,4*(i-1)-3:4*(i-1)) * F';
    mu = [mu next_mu];
    P = [P next_P];
    sigmas = [sigmas 2 .* sqrt(diag(abs(next_P)))];
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

