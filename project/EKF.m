function [P_est,x_est,S_log,ey_log,x_gt,y_log] = EKF()
    load("KFdata.mat");

    Q = [0.00035  0      0     0      0      0;
         0      0.00035  0     0      0      0;
         0      0      5.0  0      0      0;
         0      0      0     0.15  0      0;
         0      0      0     0      0.15  0;
         0      0      0     0      0      10.0];

    R = Rtrue;
    
    % estimated x0
    x0 = [10; 0; pi/2; -60; 0; -pi/2];
    
    % initial position covariance
    P_0 = [0.0005 0   0   0   0   0;
           0  0.0005 0   0   0   0;
           0  0   0.005   0   0   0;
           0  0   0   0.0005 0   0;
           0  0   0   0   0.0005 0;
           0  0   0   0   0   0.005]; % initial covariance, need to tune

    P_p = P_0;        % covariance after update step (P-minus)
    P_m = zeros(6);   % covariance after measurement step (P-plus)
    P_est = zeros(6,6000);       % log of covariance matrices at each timestep
    
    L = 0.5;          % UGV wheelbase (m)
    deltaT = tvec(2)-tvec(1);

    % need to load logged measurements and controls here
    % (or whatever it turns out to be)
    ey_log = zeros(5,1000); % log of measurement errors
    y_log = zeros(5,1000);
    S_log = zeros(5,5000);

    x_hat_p = x0; % set initial state estimate equal to initial state
                  % may need to actually estimate this later
    x_est = zeros(6,1001);
    x_est(:,1) = x_hat_p; % log of estimated system states at each timestep
    
    % get perturbation to groundtruth x
    % sampled using initial covariance
    x_perturb = mvnrnd(zeros(1,6),P_0);
    
    x_gt = zeros(6,1001);
    x_gt(:,1) = x0 + x_perturb';
    
    u = [2 -pi/18 12 pi/25];
    
    % generate groundtruth states
    for i = 2:1001
        wk = mvnrnd(zeros(1,6),Qtrue);
        [~,next_x_gt] = ode45(@motionEqs, [0.0 deltaT], x_gt(:,i-1)', [], u');
        next_x_gt = next_x_gt(end,:)' + wk';
        next_x_gt(3) = constrainAngle(next_x_gt(3));
        next_x_gt(6) = constrainAngle(next_x_gt(6));
        x_gt(:,i) = next_x_gt;
    end
    
    for i = 2:1001 %2:size(tvec,2)

        % update step
        % calculate new state and covariance from system dynamics

        % get new state estimate using numerical integration
        [~,ode_x] = ode45(@motionEqs, [0.0 deltaT], x_hat_p', [], u');
        x_hat_m = ode_x(end,:)';
        x_hat_m(3) = constrainAngle(x_hat_m(3));
        x_hat_m(6) = constrainAngle(x_hat_m(6));
        
        % update estimated state covariance
        % Not using omega matrix because it's I in this case

        %[F,G] = getLinStateMats(x_hat_p, u, L, deltaT);
        [F,G] = getLinStateMats(x_hat_m, u, L, deltaT);
        P_m = F*P_p*F' + Q;
        
        % get actual measurement
        vk = mvnrnd(zeros(1,5),Rtrue);
        y = getMeas(x_gt(:,i)) + vk';
        y_log(:,i) = y;
        
        %y = ydata(:,i);
        
        % get predicted measurement using nonlinear model  
        y_hat = getMeas(x_hat_m);

        % get measurement error
        e_y = y - y_hat;
        
        % get linearized H matrix
        H = getLinHMat(x_hat_m);
        
        %lin_ey = y - H*x_hat_m;
            
        ey_log(:,i-1) = e_y;
            
        S = H*P_m*H' + R;
        S_log(:,5*(i-2)+1:5*(i-2)+5) = S;
 
        % get the Kalman gain
        % assumes time-invariant R
        K = P_m*H'/S;

        % get corrected state and covariance estimates
        x_hat_p = x_hat_m + K*e_y;
        P_p = (eye(6) - K*H)*P_m;
        
        x_hat_p(3) = constrainAngle(x_hat_p(3));
        x_hat_p(6) = constrainAngle(x_hat_p(6));
        
        x_est(:,i) = x_hat_p; % add newly estimated state to log
        P_est(:,6*(i-2)+1:6*(i-2)+6) = P_p;

    end
    
end