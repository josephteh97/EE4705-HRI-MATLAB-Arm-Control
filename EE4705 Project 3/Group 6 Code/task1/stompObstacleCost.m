function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)

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

dxb = voxel_world_sEDT(sub2ind(world_size, idx(:, 1), idx(:, 2), idx(:, 3)));
dist_array = safety_margin + radius - dxb;
zero_array = zeros(size(dist_array),'like', dist_array);
cost_array = max(dist_array, zero_array) .* abs(vel);
cost = sum(cost_array);

end