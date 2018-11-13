clear; 

k = 50;
f = 0.8;
q = 10;
sigma = 10;

for i = 1:k
    sigma = sigma*f^2+q;
end

sigma_p = q / (1 - f^2);