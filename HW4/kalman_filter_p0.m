% Homework 4 Part 0
% Kalman Filter for Center of Mass Location and Velocity Estimation

% load marker data
markerData = load('part0/p1n00');

% initialize variables and variances
xk = zeros(6,1);
Q = diag(ones(6,1))*(1*10^-8);
Pk = diag(ones(6,1))*(1*10^-2);
H = zeros(24,6);
R = diag(ones(24,1))*(1*10^-2);
states = zeros(length(markerData),6);

% compute prediction matrix
F = eye(6,6);
for n = 1:3
   F(n,n+3) = 0.1; 
end

% encode sensor readings
for n = 1:3:24
    for i = 1:3
        H((n-1)+i,i) = 1;
    end
end

% do Kalman filtering
for n = 1:length(markerData)
   % predict state and covariance
   xk = F*xk;
   Pk = F*Pk*F' + Q;
   
   % use noisy data to find residuals and residual covariances
   z = markerData(n,:)';
   y = z - H*xk;
   K = Pk*H'*inv(H*Pk*H' + R);
   
   % update predictions based on measured data
   xk = xk + K*y;
   Pk = (eye(size(K,1)) - K*H)*Pk;
   states(n,:) = xk;
end