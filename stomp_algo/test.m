mu = [2 3];
Sigma = [1 1.5; 1.5 3];
rng('default')  % For reproducibility
R = mvnrnd(mu,Sigma,100);

R{1} = zeros(100, 1);