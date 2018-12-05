
Q = 0.01*eye(6); % process noise prior, same for all terms for now
R = 0.01*eye(5); % measurement noise prior, same for all terms for now

P_0 = 0.01*eye(6); % initial covariance, need to tune
P_m = P0;          % covariance after update step (P-minus)
P_p = zeros(6);    % covariance after measurement step (P-plus)

t = 0;        % current time
deltaT = 0.1; % timestep length (s)
L = 0.5;      % UGV wheelbase (m)

% need to load logged measurements and controls here
% (or whatever it turns out to be)
y_log = [];
u_log = [];

x0 = [10; 0; pi/2; -60; 0; -pi/2];

x_hat = x0; % set initial state estimate equal to initial state
            % may need to actually estimate this later
x_est = []; % log of estimated system states for debugging

% just run for 100 timesteps for now
% this will definitely change
for i = 1:100
    
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
    % TO DO: figure out how to calculate Omega matrix
    % assumes time invariant process noise 
    P_m = F*P*F' + Omega*Q*Omega';
    
    % TO DO: make function to get measurements
    y = getMeasurement();
    if (~isEmpty(y))
        
        % get predicted measurement using nonlinear model
        y_hat = [atan((x_hat_m(5)-x_hat_m(2))/(x_hat_m(4)-x_hat_m(1)))-x_hat_m(3);
                 sqrt((x_hat_m(1)-x_hat_m(4))^2+(x_hat_m(2)-x_hat_m(5))^2);
                 atan((x_hat_m(2)-x_hat_m(5))/(x_hat_m(1)-x_hat_m(5)))-x_hat_m(6);
                 x_hat_m(4);
                 x_hat_m(5)];
        
        % get measurement error
        e_y = y - y_hat;
        
        % get linearized H matrix
        H = getLinHMat(x_hat_m);
        
        % get the Kalman gain
        % assumes time-invariant R
        K = P_m*H'/(H*P_m*H'+R);
        
        % get corrected state and covariance estimates
        % TO DO: verify dim of identity matrix
        x_hat_p = x_hat_m + K*e_y;
        P_p = (eye(6) - K*H)*P_m;
    else
        x_hat_p = x_hat_m;
        P_p = P_m;
    end
    
    t = t + deltaT; % advance time
    x_est = [x_est x_hat_p]; % add newly estimated state to log
end