function [ node ] = downsampling( pointcloud ,grid_size)
%===========================================%
%   Downsampling pointcloud by grid filter
%   Method:     Create grid evenly in the space, assign each point to its
%               corresponding grid. Then for each grid, if there's more
%               than one point, eliminate all except one. Record the index
%   Input:      pointcloud:     ...
%               grid_size:      Threshold of grid size
%   Output:     node:  N¡Á3. node positions
%===========================================%
if(isempty(pointcloud))
    node = [];
    return;
end
%   Calculate the boundry of the volume
max_grid_x = max(pointcloud(:,1));
max_grid_y = max(pointcloud(:,2));
max_grid_z = max(pointcloud(:,3));
min_grid_x = min(pointcloud(:,1));
min_grid_y = min(pointcloud(:,2));
min_grid_z = min(pointcloud(:,3));

%   Calculate grid number
num_grid_x = floor((max_grid_x - min_grid_x) / grid_size) + 1;
num_grid_y = floor((max_grid_y - min_grid_y) / grid_size) + 1;
num_grid_z = floor((max_grid_z - min_grid_z) / grid_size )+ 1;

%   point cloud storage. col 1:index;  col 2: grid number
point_cloud_storage      = zeros(size(pointcloud,1),2);
point_cloud_storage(:,1) = [1:1:size(pointcloud,1)]';
for i = 1 : size(pointcloud,1)
    temp_index_x = floor((pointcloud(i,1) - min_grid_x) / grid_size) + 1;
    temp_index_y = floor((pointcloud(i,2) - min_grid_y) / grid_size) + 1;
    temp_index_z = floor((pointcloud(i,3) - min_grid_z) / grid_size) + 1;
    point_cloud_storage(i,2) = sub2ind([num_grid_x num_grid_y num_grid_z]...
                                        ,temp_index_x,temp_index_y,temp_index_z);
end

point_cloud_storage = sortrows(point_cloud_storage,2);
temp = point_cloud_storage(1,2);
i = 2;
start_line = 1;     %   Randomly chosen points
end_line   = -9999; %   Randomly chosen points
node = [];
while(i<=size(point_cloud_storage,1)-5)
    if(point_cloud_storage(i,2)==temp)
        i = i + 1;
    else
        end_line   = i - 1;
        if(start_line == end_line)      %   Only one point exist in the grid
            centerpoint = pointcloud(point_cloud_storage(start_line:end_line,1),:);
        else
            centerpoint = mean(pointcloud(point_cloud_storage(start_line:end_line,1),:));
        end
        node = [node;centerpoint];
        start_line = i;
        temp = point_cloud_storage(i,2);
    end    
end

%node = pointcloud(chosen_points(:,1),:);
end

