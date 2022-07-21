function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)
% only used in stomp trajectory cost
safety_margin = 0.05; % the safety margin distance, unit: meter
cost = 0;
% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;
% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
% in the voxel world.
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);

% Eq (13) in the STOMP conference paper. 
for b = 1 : length(radius)
        
        temp = max(safety_margin + radius(b) - voxel_world_sEDT(idx(b, 1), idx(b, 2), idx(b, 3)), 0) * abs(vel(b));
        cost = cost + temp;

end

end