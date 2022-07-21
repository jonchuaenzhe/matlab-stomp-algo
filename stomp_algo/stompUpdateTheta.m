function [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M)

% Smoothed delta theta
dtheta_smoothed = M * dtheta(:, 2:end - 1)';

% Update theta
theta(:,2:end-1) = theta(:,2:end-1) + dtheta_smoothed';

end