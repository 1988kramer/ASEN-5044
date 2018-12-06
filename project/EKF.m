function [P_est,x_est,S_log,ey_log,x_gt] = EKF()
    load("KFdata.mat");
    Q = Qtrue;
    R = Rtrue;
    %Q = 0.01*eye(6); % process noise prior, same for all terms for now
    %R = 0.01*eye(5); % measurement noise prior, same for all terms for now

    P_0 = 0.1*eye(6); % initial covariance, need to tune
    P_p = P_0;          % covariance after update step (P-minus)
    P_m = zeros(6);    % covariance after measurement step (P-plus)
    P_est = [];        % log of covariance matrices at each timestep
    
    L = 0.5;      % UGV wheelbase (m)
    deltaT = tvec(2)-tvec(1);

    % need to load logged measurements and controls here
    % (or whatever it turns out to be)
    ey_log = []; % log of measurement errors
    S_log = [];
    
    x0 = [10; 0; pi/2; -60; 0; -pi/2];

    x_hat_p = x0; % set initial state estimate equal to initial state
                  % may need to actually estimate this later
    x_est = x0; % log of estimated system states at each timestep
    
    x_gt = x0; 

    for i = 2:size(tvec,2)

        % update step
        % calculate new state and covariance from system dynamics

        u = [2 -pi/18 12 pi/25];

        % get linearized matrices
        % use a different x_nom here if available
        [F,G] = getLinStateMats(x_hat_p, u, L, deltaT);

        % get new state estimate using numerical integration
        [~,ode_x] = ode45(@motionEqs, [0.0 deltaT], x_hat_p', [], u');
        x_hat_m = ode_x(end,:)';
        
        % get next x groundtruth
        [~,next_x_gt] = ode45(@motionEqs, [0.0 deltaT], x_gt(:,i-1)', [], u');
        % add process noise to groundtruth;
        next_x_gt = mvnrnd(next_x_gt(end,:), Qtrue); 
        x_gt = [x_gt next_x_gt'];
        
        % update estimated state covariance
        % Not using omega matrix because it's I in this case
        % assumes time invariant process noise 
        P_m = F*P_p*F' + Q;

        % get actual measurement
        y = mvnrnd(getMeas(next_x_gt),Rtrue);
        y = y';
        
        % get predicted measurement using nonlinear model  
        y_hat = getMeas(x_hat_m);

        % get measurement error
        e_y = y - y_hat;
            
        ey_log = [ey_log e_y];

        % get linearized H matrix
        H = getLinHMat(x_hat_m);
            
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