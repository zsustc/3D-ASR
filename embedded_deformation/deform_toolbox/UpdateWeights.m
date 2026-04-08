function [ weights ] = updateWeights(pointcloud1,pointcloud2,num_nearestpts)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: pointcloud2 are reference point cloud, for each point in
%             pointcloud1, calculate its weight to "num_nearestpts"
%             reference points in pointcloud1 based on distance
%   Method:   weight = distance(i,j) / total_distance
%   Input:    pointcloud1:     Predicted depth and nodes. N¡Á3
%             pointcloud2:     Current depth image        M¡Á3
%             num_nearestpts:  number of nearest points to pointcloud2
%   Returns:  weight:          N¡Á2*num_nearestpts. Each row is each point
%                              in pointcloud1, odd column is the index to
%                              pointcloud2, even column is the weight.
%                              All weights are norminalized.
%   Author:   Jingwei Song.   12/06/2016 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% standard data structure for weight is:
% [index_point 1,weight 1,index_point 2,weight 2,index_point 3,weight 3]
% [2,  0.6,  1,  0.3,  3,  0.1]
weights = zeros(size(pointcloud1,1),num_nearestpts*2);

num_mapping_node  = size(pointcloud2,1);
for i = 1 : size(pointcloud1, 1)
    point = pointcloud1(i,:);
    
    % data structure for weight_temp
    % [index_point 1,distance 1,index_point 2,distance 2,index_point 3,distance 3,   .....]
    % [1,  3;  2,  2;  3,  1;   .....]
    weight_temp = zeros(num_mapping_node,2);
    weight_temp(:,2) = 1 / num_mapping_node;
    
    for j = 1 : size(pointcloud2, 1)
        nodepoint = pointcloud2(j,:);
        weight_temp(j,1) = j;
        weight_temp(j,2) = sqrt((point(1,1) - nodepoint(1,1))^2 + (point(1,2) - nodepoint(1,2))^2 + (point(1,3) - nodepoint(1,3))^2);
    end
    weight_temp = sortrows(weight_temp,2);     %sort distance accendingly
    
    total_weight_temp = 0;           %for norminalization later
    for j = 1 : num_nearestpts
        weight_temp(j,2) = (1 - weight_temp(j,2) / weight_temp(num_nearestpts + 1,2));
        total_weight_temp = total_weight_temp + weight_temp(j,2);
    end
    % weight norminalization
    for j = 1 : num_nearestpts
        weights(i,2*j-1) = weight_temp(j,1);
        weights(i,2*j) = weight_temp(j,2) / total_weight_temp;
    end
end

end

