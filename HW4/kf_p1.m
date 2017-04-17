function states = kf_p1(fn)
% Program used with quick_write.m to make the homework go faster

% load marker data
markerData = load(fn);

% initialize variables and variances
xk = zeros(12,1);
Q = diag(ones(12,1))*(1*10^-8);
Pk = diag(ones(12,1))*(1*10^-2);
H = zeros(24,12);
R = diag(ones(24,1))*(1*10^-2);
states = zeros(length(markerData),13);

% compute prediction matrix
F = eye(12,12);
for n = [1:3,7:9]
   F(n,n+3) = 0.1; 
end

% encode sensor readings
for n = 1:3:24
    for i = 1:3
        H((n-1)+i,i) = 1;
        H((n-1)+i,i+6) = pi/180;
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
   K = (Pk*H')/(H*Pk*H' + R);
   
   % update predictions based on measured data
   xk = xk + K*y;
   Pk = (eye(size(K,1)) - K*H)*Pk;
   states(n,:) = [xk(1:6)', eul2quat(flip(xk(7:9)')), xk(10:12)'];
end
end