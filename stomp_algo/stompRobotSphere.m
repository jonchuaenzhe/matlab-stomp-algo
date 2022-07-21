%% Construct the sphere around the robot maniputor
% It requires knowledge of the robot geometry. Here we roughly set the
% sphere radius to be 0.05m to cover the KINOVA Gen3 robot. There are 7
% joints and 8 links of KINOVA Gen3.
% INPUT:
%       X: joints (x,y,z,1) position in the world frame (nJoints by 4)
% OUTPUT:
%       
function [centers, radi] = stompRobotSphere(X)
nJoints = size(X,1);
center_cell = cell(nJoints,1);
radi_cell = cell(nJoints,1);
% consturct the spheres for all links
for k=1:size(X,1)
    if k==1
        parent_joint_position = [0,0,0]; % The base coordinate
    else
        parent_joint_position = X(k-1, 1:3);
    end
    child_joint_poisition = X(k, 1:3);
    % number of spheres used to cover the kth link
    rad = 0.05;  % radius of the sphere, tuning parameters 
    % calculate the number of shperes to cover the link
    nSpheres = ceil(norm(child_joint_poisition-parent_joint_position)/rad)+1; % one sphere every 0.05m
    % Calculate the centers of the spheres
    center_cell_k = arrayfun(@(x1, x2) linspace(x1,x2,nSpheres), parent_joint_position', child_joint_poisition','UniformOutput', false);
    % xyz coordinates of each sphere
    center_cell{k} = cell2mat(center_cell_k)';
    % radius of each sphere
    radi_cell{k} = rad*ones(size(center_cell{k},1),1);
end    

centers = cell2mat(center_cell);
radi = cell2mat(radi_cell);

end