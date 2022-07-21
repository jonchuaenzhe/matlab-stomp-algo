%% Construct voxel obstacle representation for STOMP planner
% define the workspace (the voxel world limits). Depending on the robot's reach range.
Env_size = [-1, -1, -1; 2, 2, 2];  % [xmin, ymin, zmin] for 1st row, xyz-lengths for 2nd row
voxel_size = [0.02, 0.02, 0.02];  % unit: m
% Binary map: all free space initially, 
binary_world = zeros(Env_size(2, 1) / voxel_size(1), Env_size(2, 2) / voxel_size(2), Env_size(2, 3) / voxel_size(3));
% binary_world_offset = Env_size(1, :)./ voxel_size;
%% XYZ metric representation (in meter) for each voxel 
% !!!!Watch out for the useage of meshgrid:
% [X,Y,Z] = meshgrid(x,y,z) returns 3-D grid coordinates defined by the 
% vectors x, y, and z. The grid represented by X, Y, and Z has size 
% length(y)-by-length(x)-by-length(z).
% The 3D coordinate is of the center of the voxels
[Xw, Yw, Zw] = meshgrid(Env_size(1, 1) + 0.5 * voxel_size(1) : voxel_size(1) : Env_size(1, 1) + Env_size(2, 1) - 0.5 * voxel_size(1), ...
       Env_size(1, 2) + 0.5 * voxel_size(2) : voxel_size(2) : Env_size(1, 2) + Env_size(2, 2) - 0.5 * voxel_size(2), ...
    Env_size(1, 3) + 0.5 * voxel_size(3) : voxel_size(3) : Env_size(1, 3) + Env_size(2, 3) - 0.5 * voxel_size(3));

%% Static obstacles

% Task 1 coordinates
lbox = 0.08*2; % length of the cube
box_center = [0.4 0.46 0.26]; % (metric) world coordinates of the box center

% Task 3 own coordinates
%lbox = 0.08*2; % length of the cube
%box_center = [0.4 0.5 -0.1]; % (metric) world coordinates of the box center

aObs = collisionBox(lbox,lbox,lbox);  
aObs.Pose = trvec2tform(box_center);   

% Obstacle 2
%box_center = [0.4 0.5 0.5]; % (metric) world coordinates of the box center

%bObs = collisionBox(lbox,lbox,lbox);  
%bObs.Pose = trvec2tform(box_center);   

% Set static obstacles
world = {aObs}; % try adding more static obstacles aObs to dObs or create your own

%% voxelize the box obstacles
cube_metric = [box_center-lbox;
                lbox,lbox,lbox]; % [xmin, ymin, zmin] for 1st row, xyz-lengths for 2nd row
% range (lower and upper limits) of the cube voxel
% The number inside ceil is always positive
cube_voxel = [ceil((cube_metric(1, :)-Env_size(1,:))./voxel_size); ...
    ceil((cube_metric(1, :) + cube_metric(2, :)-Env_size(1,:))./voxel_size)];

% Update the cube occupancy in the voxel world
% First generate the 3D grid coordinates (x,y,z subcripts) of cube in the voxel world
[xc, yc, zc] = meshgrid(cube_voxel(1, 1):cube_voxel(2, 1), cube_voxel(1, 2):cube_voxel(2, 2), cube_voxel(1, 3):cube_voxel(2, 3));
% Set the corresponding voxel occupancy to 1, meaning occupied
binary_world(sub2ind([Env_size(2, 1) / voxel_size(1), Env_size(2, 2) / voxel_size(2), Env_size(2, 3) / voxel_size(3)], xc, yc, zc)) = 1;

% % plot the occupied voxel with a marker *
% plot3(xc(:), yc(:), zc(:), '*');
% % Or you can use the volumeViewer() from the Image Processing Toolbox to 
% % display the voxel_world in 3D. 
% volumeViewer(voxel_world);

%% construct signed Euclidean Distance for the voxel world
% Only approximation if the voxel is not a cube
voxel_world_sEDT = prod(voxel_size) ^ (1/3) * sEDT_3d(binary_world);


voxel_world.voxel_size = voxel_size;
voxel_world.voxel = binary_world;
% voxel_world.offset =  binary_world_offset;
voxel_world.world_size = size(binary_world);
voxel_world.Env_size = Env_size; % in metric 
voxel_world.sEDT =  voxel_world_sEDT;





