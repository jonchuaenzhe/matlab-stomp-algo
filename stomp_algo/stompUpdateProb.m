% Compute the exp(-1/lambda * S{ki}) form for trajectory probability
% Equation (11) in the ICRA 2011 paper

function trajProb = stompUpdateProb(Stheta)
% sensitivity
h = 10;

% max and min local cost at each time step
maxS = max(Stheta, [], 1);
minS = min(Stheta, [], 1);

% exp calculates the element-wise exponential
expCost = exp(-h * (Stheta- minS) ./ (maxS - minS));

% To handle the case where maxS = minS = 0. This is possible when local
% trajectory cost only includes obstacle cost, which at the end of the time
% duration, the speed is 0, or when there is no collision.
expCost(isnan(expCost) == 1) = 0;
% normalize the exponentialized cost to have the probabilities
trajProb = expCost./ sum(expCost, 1);
trajProb(isnan(trajProb) == 1) = 0;

end