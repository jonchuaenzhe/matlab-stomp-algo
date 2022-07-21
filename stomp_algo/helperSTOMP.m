%Parameters
% T = 5;
nDiscretize = 20; % number of discretized waypoint
nPaths = 20; % number of sample paths
convergenceThreshold = 0.1; % convergence threshhold

% Initial guess of joint angles theta is just linear interpolation of q0
% and qT
q0 = currentRobotJConfig;
qT = finalRobotJConfig;
numJoints = length(q0);
theta=zeros(numJoints, nDiscretize);
for k=1:length(q0)
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end

% by default, it loads the robot with the structure data format
robot_struct = loadrobot(robot_name); 

% store sampled paths
theta_samples = cell(1,nPaths);

%% for calculating the acceleration of theta
% Precompute
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1); 
R = A' * A;
Rinv = inv(R);
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1); % normalized by each column, no longer symmetric
Rinv = 1.5*Rinv/sum(sum(Rinv)); % normalized R inverse, so that the sample is still within the voxel world

%% Task 5
Tlist = zeros(4,4,7);
for i=1:numJoints % Store all the homogeneous transform matrices
    Tlist(:,:,i) = getTransform(robot_struct, robot_struct.homeConfiguration, robot_struct.BodyNames{i});
end

%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

%%% --------- updateJointsWorldPosition needed for stompTrajCost
[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world, Tlist);
QthetaOld = 0;

iter=0;
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1;
    % overall cost: Qtheta
    QthetaOld = Qtheta;
    % use tic and toc for printing out the running time
    %tic
    % Sample noisy trajectories -> STEP 1
    %%% ---------- stompSamples for generating noisy trajectories
    [theta_paths, em]=stompSamples(nPaths, Rinv, theta);
    %disp("STEP 1 DONE")

    % Calculate Local trajectory cost -> STEP 2A
    Stheta = zeros(nPaths, nDiscretize);
    %%% ++++++++++ stompTrajCost on loop for generating Stheta
        %%% ---------- updateJointsWorldPosition used in stompTrajCost
        %%% ---------- stompObstacleCost used in stompTrajCost
    tic
    for k = 1 : nPaths
        
        [S, Q] = stompTrajCost(robot_struct, theta_paths{k}, R, voxel_world, Tlist);
        Stheta(k, :) = S;

    end
    toc
    %disp("STEP 2A DONE")
    
    % Given local traj cost, update local traj probability -> STEP 2B
    %%% ++++++++++ stompUpdateProb to calculate Ptheta
    trajProb = stompUpdateProb(Stheta);
    %disp("STEP 2B DONE")
    
    % Compute delta theta (aka gradient estimator) -> STEP 3
    %%% ---------- stompDthetha to output dtheta
    dtheta = stompDTheta(trajProb, em);
    %disp("STEP 3 DONE")

    % Compute the cost of the new trajectory -> STEP (4 & 5), 6
    %%% ++++++++++ stompUpdateThetha to output new theta
    %%% ++++++++++ stompTrajCost to calculate the Qtheta
    [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M);
    [~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world, Tlist);
    %disp("ALL DONE")
 
    %toc

    Q_time = [Q_time Qtheta];
    % control cost
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta % display overall cost
    RAR  % display control cost

    % Stop iteration criteria:
    if iter > 30 || sum(dtheta_smoothed,'all') == 0
        disp('Maximum iteration (30) has reached.')
        break
    end

    if sum(dtheta_smoothed,'all') == 0
        disp('Estimated gradient is 0.')
        break
    end

end

disp('STOMP Finished.');
disp(iter);

%% Plot path
% axis tight manual % this ensures that getframe() returns a consistent size
for t=1:size(theta,2)
    show(robot, theta(:,t),'PreservePlot', true, 'Frames', 'on');
    drawnow;
    pause(1/50);
end

%% Show video - uncomment to get video

%v = VideoWriter('Kuka7.avi');
%v.FrameRate =2;
%open(v);

%for t=1:size(theta,2)
%    show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
%    drawnow;
%    frame = getframe(gcf);
%    writeVideo(v,frame);
%    pause(10/20);
%%     pause;
%end
%close(v);


%% save data
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')
