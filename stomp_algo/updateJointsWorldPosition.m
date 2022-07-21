%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta, Tlist)
% To further optimise the PoE code, remove robot_struct as a variable in
% the function as it is not used.

% In this sample code, we directly call the MATLAB built-in function getTransform
% to calculate the forward kinemetics

% Update the robot configuration structure used by Matlab
% Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
theta_cell = num2cell(theta);
% Because the getTranform() function can only takes in structure array
% robot Configuration, we have to construct it first by copying the structure from the homeConfiguration
% robot_struct = loadrobot(robot_name); % by default, it loads the robot with structure data format
tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
[tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta
% get the number of joints
nJoints = length(theta);
T = cell(1,nJoints);
X = zeros(nJoints, 4);

%% Task 5
% ------------ PoE method -------------
S = zeros(6,nJoints); % screw axis list, S list

for k=1:nJoints
    % get the homegeneous transformation from kth joint's frame to the
    % base frame
    % getTransform can only takes in structure array Configuration
    %T{k}= getTransform(robot_struct, tConfiguration, robot_struct.BodyNames{k}); 
    
    % from inspection, we know that all screw axes are along the z-axis
    % so we automate getting the screw axes as below:
    [R, p] = TransToRp(Tlist(:,:,k)); % rotation matrix R and position vector p
    w = R(:,3);
    v = cross(p, w); % - s x q + hs, but h = 0
    S(:,k) = [w; v]; % add to S list
    
    %% apply POE formula
    T{k} = FKinSpace(Tlist(:,:,k), S(:,1:k), theta(1:k));
    % Get joint's world coordinates
    X(k,:) = transpose(T{k} * [0;0;0;1]);

end

%% automate finding screw axis
% This method takes longer than the method shown above, though it is more
% transferrable and optimised.

%     theta_joint = zeros(1,nJoints);
%     theta_joint(1,k)=theta(k); % all other joint angles should be zero
%     
%     theta_cell = num2cell(theta_joint);
%     tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
%     [tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta
% 
%     T1 = Tlist(:,:,k); % home configuration
%     T2 = getTransform(robot_struct, tConfiguration, robot_struct.BodyNames{k}); % desired configuration
%     S(:,k) = se3ToVec(MatrixLog6(T2/T1))/theta(k); % normalize the spatial twist to get screw axis

end