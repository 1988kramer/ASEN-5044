function [P_est,x_est,S_log,ey_log,x_gt] = EKF()
    load("KFdata.mat");
    % best Q so far
    Q = [0.55 0 0 0 0 0;
         0 0.77 0 0 0 0;
         0 0 1.70 0 0 0;
         0 0 0 0.85 0 0;
         0 0 0 0 0.92 0;
         0 0 0 0 0 1.66];

    %Q = [0.05  0     0    0     0     0;
    %     0     0.05  0    0     0     0;
    %     0     0     2.5  0     0     0;
    %     0     0     0    0.75  0     0;
    %     0     0     0    0     0.75  0;
    %     0     0     0    0     0     2.5];
    R = [2.85 0    0    0     0;
         0    41.8 0    0     0;
         0    0    8.15 0     0;
         0    0    0    20.88 0;
         0    0    0    0     17.2944];
    %R = Rtrue;
    %Q = zeros(6);
    %Q = 0.25 * Q;
    
    % try better method to estimate x0 and P0
    x0 = [10; 0; pi/2; -60; 0; -pi/2];
    perturb_x0 = [0; 1; 0; 0; 0; 0.1];
    P_0 = [0.0005 0   0   0   0   0;
           0  0.0005 0   0   0   0;
           0  0   0.005   0   0   0;
           0  0   0   0.0005 0   0;
           0  0   0   0   0.0005 0;
           0  0   0   0   0   0.005]; % initial covariance, need to tune
    %P_0 = 1 * eye(6);
    P_p = 1.0 * P_0;        % covariance after update step (P-minus)
    P_m = zeros(6);   % covariance after measurement step (P-plus)
    P_est = [];       % log of covariance matrices at each timestep
    
    L = 0.5;          % UGV wheelbase (m)
    deltaT = tvec(2)-tvec(1);

    % need to load logged measurements and controls here
    % (or whatever it turns out to be)
    ey_log = []; % log of measurement errors
    S_log = [];

    x_hat_p = x0; % set initial state estimate equal to initial state
                  % may need to actually estimate this later
    x_est = x_hat_p; % log of estimated system states at each timestep
    
    x_pert = mvnrnd(zeros(1,6),P_0);
    x_gt = x0 + x_pert'; 

    for i = 2:1001 %2:size(tvec,2)

        % update step
        % calculate new state and covariance from system dynamics

        u = [2 -pi/18 12 pi/25];

        % get new state estimate using numerical integration
        [~,ode_x] = ode45(@motionEqs, [0.0 deltaT], x_hat_p', [], u');
        x_hat_m = ode_x(end,:)';
        x_hat_m(3) = constrainAngle(x_hat_m(3));
        x_hat_m(6) = constrainAngle(x_hat_m(6));
        
        % get next x groundtruth
        [~,next_x_gt] = ode45(@motionEqs, [0.0 deltaT], x_gt(:,i-1)', [], u');
        % add process noise to groundtruth;
        wk = mvnrnd(zeros(1,6),Qtrue);
        next_x_gt = next_x_gt(end,:) + wk;

        next_x_gt(3) = constrainAngle(next_x_gt(3));
        next_x_gt(6) = constrainAngle(next_x_gt(6));
        x_gt = [x_gt next_x_gt'];
        
        % update estimated state covariance
        % Not using omega matrix because it's I in this case

        %[F,G] = getLinStateMats(x_hat_p, u, L, deltaT);
        [F,G] = getLinStateMats(x_hat_m, u, L, deltaT);
        P_m = F*P_p*F' + Q;

        % get actual measurement
        vk = mvnrnd(zeros(1,5),Rtrue);
        y = getMeas(next_x_gt) + vk';
        %y = getMeas(next_x_gt);
        
        % get predicted measurement using nonlinear model  
        y_hat = getMeas(x_hat_m);

        % get measurement error
        e_y = y - y_hat;
        
        % get linearized H matrix
        H = getLinHMat(x_hat_m);
        
        %lin_ey = y - H*x_hat_m;
            
        ey_log = [ey_log e_y];
            
        S = H*P_m*H' + R;
        S_log = [S_log S];
 
        % get the Kalman gain
        % assumes time-invariant R
        K = P_m*H'/S;

        % get corrected state and covariance estimates
        x_hat_p = x_hat_m + K*e_y;
        P_p = (eye(6) - K*H)*P_m;
        
        x_hat_p(3) = constrainAngle(x_hat_p(3));
        x_hat_p(6) = constrainAngle(x_hat_p(6));
        
        x_est = [x_est x_hat_p]; % add newly estimated state to log
        P_est = [P_est P_p];

    end
end