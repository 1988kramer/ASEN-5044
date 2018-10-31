function [x,P] = recursive(y)
    x = [];
    P = [];
    H = eye(3);
    R = [8 5.15 6.5;
         5.15 5 -4.07;
         6.5 -4.07 50];
    P_i = eye(3,3) * 1e2;
    x_i = [0;0;0];
    for i = 1:size(y,2)
        x = [x x_i];
        P = [P P_i];
        
        K = P_i * H' / (H*P_i*H' + R);
        x_i = x_i + K * (y(:,i)-H*x_i);
        P_i = (eye(3)-K*H)*P_i*(eye(3)-K*H)' + K*R*K';
    end
    x = [x x_i];
    P = [P P_i];
end