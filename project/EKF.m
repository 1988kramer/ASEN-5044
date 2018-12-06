function [P_est,x_est,ey_log,S_log] = EKF()
    load("KFdata.mat");

    Q = 0.01*eye(6); % process noise prior, same for all terms for now
    R = 0.01*eye(5); % measurement noise prior, same for all terms for now

    P_0 = 0.01*eye(6); % initial covariance, need to tune
    P_m = P0;          % covariance after update step (P-minus)
    P_p = zeros(6);    % covariance after measurement step (P-plus)
    P_est = [];        % log of covariance matrices at each timestep
    
    L = 0.5;      % UGV wheelbase (m)
    deltaT = t(2)-t(1);

    % need to load logged measurements and controls here
    % (or whatever it turns out to be)
    ey_log = []; % log of measurement errors
    S_log = [];
    
    x0 = [10; 0; pi/2; -60; 0; -pi/2];

    x_hat = x0; % set initial state estimate equal to initial state
                % may need to actually estimate this later
    x_est = []; % log of estimated system states at each timestep

    for i = 2:size(tvec,2)

        % update step
        % calculate new state and covariance from system dynamics

        u = u_log(:,i);

        % get linearized matrices
        % use a different x_nom here if available
        [F,G] = getLinStateMats(x_hat, u, L, deltaT);

        % get new state estimate using numerical integration
        [t,ode_x] = ode45(@motionEqs, [0.0 deltaT], x_hat_p', [], u');
        x_hat_m = ode_x(end,:)';

        % update estimated state covariance
        % Not using omega matrix because it's I in this case
        % assumes time invariant process noise 
        P_m = F*P*F' + Q;

        % TO DO: make function to get measurements
        y = ydata(:,i);
        if (~isEmpty(y))
            
            d52 = x_hat_m(5) - x_hat_m(2);
            d41 = x_hat_m(4) - x_hat_m(1);
            d14 = -d41;
            d25 = -d52;

            % get predicted measurement using nonlinear model
            y_hat = [atan2(d52,d41)-x(i+1,3);
                     sqrt(d14^2+d25^2);
                     atan2(d25,d14)-x(i+1,6);
                     x(i+1,4);
                     x(i+1,5)];

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
            % TO DO: verify dim of identity matrix
            x_hat_p = x_hat_m + K*e_y;
            P_p = (eye(6) - K*H)*P_m;
        else
            x_hat_p = x_hat_m;
            P_p = P_m;
        end
        
        x_est = [x_est x_hat_p]; % add newly estimated state to log
        P_est = [P_est P_p];
    end
end