% update theta 
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);

dtheta = zeros(nJoints, nDiscretize);
% iterate over all joints
for m = 1 : nJoints
    
    % take the m-th joint of em (path increment)
    % em{m} is (nSamplePaths, nDiscretize)
    % transpose to get (nDiscretize, nSamplePaths)
    e = em{m}';
    
    for n = 1 : nDiscretize
        
        % e(n,:) is (1, nSamplePaths)
        % trajProb(:, n) is (nSamplePaths, 1)
        % multiply them is dot product to get a single value
        dtheta(m, n) = e(n,:) * trajProb(:, n);

    end

end