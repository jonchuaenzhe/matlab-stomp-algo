% Input: 
%   sigma： sample covariance matrix
%   theta: mean trajectory from last iteration
% Output:
%   theta_paths: sampled trajectories
%   em: sampled Gaussian trajectory for each joint

function [theta_paths, em]=stompSamples(nSamplePaths,sigma,theta)
% Sample theta (joints angles) trajectory 

[nJoints, nDiscretize] = size(theta);

em = cell(1,nJoints);
ek = cell(1,nSamplePaths);

theta_paths = cell(1, nSamplePaths);
mu=zeros(1,length(sigma)); % length is nDiscretize


for m = 1 : nJoints
    % Each joint is sampled independently
    % The starting q0 and final qT are fixed, so set the sample to 0
    % sample from multivariable Gaussian distribution

    %% -------- JON"S EDITS -----------
    
    % mu is mean = 0
    % sigma is covariance matrix = Rinv
    % use multivariate random distribution to get R (all sample path
    % increment)
    R = mvnrnd(mu,sigma,nSamplePaths);

    % set start and end to 0
    R = [zeros(nSamplePaths, 1) R];
    R = [R zeros(nSamplePaths, 1)];

    % this is for one joint. loop over all joints
    em{m} = R;

    %% ----------- END ----------------
   
end

% regroup it by samples
emk = [em{:}];
for k=1:nSamplePaths
    ek{k} = reshape(emk(k,:),nDiscretize, nJoints)';
    theta_paths{k} = theta + ek{k};
    
end