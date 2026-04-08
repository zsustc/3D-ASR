function [ pointcloud_new, ED_Parameter] = embedDeform( pointcloud,keypts_old,keypts_new,...
                                           num_nearestpts,grid_size)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Embedded deformation
%   Method:   Do embedded deformation according to inputs
%   Input:    pointcloud:       Current point cloud
%             keypts_old:       Old key points
%             keypts_new:   	New key points
%             num_nearestpts:   Number of points that share edge with this
%                               point
%             grid_size:        Downsampling grid size
%   Returns:   
%             pointcloud_new:   Deformed point cloud
%   Author:   Jingwei Song.     28/07/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%====================User input=====================%
ED_Parameter.num_nearestpts = num_nearestpts;     %for calculating weights
ED_Parameter.grid_size      = grid_size;    % Downsampling grid size
%====================================================%


% user edit controlling vertices
ED_Parameter.control_vertices.prior = keypts_old;
ED_Parameter.control_vertices.after = keypts_new;

%node.positions 
ED_Parameter.node.positions = downsampling(pointcloud ,ED_Parameter.grid_size);
ED_Parameter.num_nodes      = size(ED_Parameter.node.positions,1);
ED_Parameter.node.rotation_matrix        = zeros(ED_Parameter.num_nodes*3,3);
ED_Parameter.node.transformation_matrix  = zeros(ED_Parameter.num_nodes,3);
ED_Parameter.node.weight = UpdateWeights( ED_Parameter.node.positions,...
                    ED_Parameter.node.positions, ED_Parameter.num_nearestpts );

for i = 1 : ED_Parameter.num_nodes
    ED_Parameter.node.rotation_matrix(3*i-2:3*i,:) = speye(3,3);
end


ED_Parameter.vertices_prior.positions = ED_Parameter.control_vertices.prior;
ED_Parameter.vertices_prior.weight = UpdateWeights(ED_Parameter.vertices_prior.positions,ED_Parameter.node.positions,ED_Parameter.num_nearestpts);

x0                      = [ED_Parameter.node.rotation_matrix; ED_Parameter.node.transformation_matrix];
x_optimum               = Gauss_Newton_Optimization(x0,ED_Parameter);
ED_Parameter.node.rotation_matrix       = x_optimum(1: 3*ED_Parameter.num_nodes,:);
ED_Parameter.node.transformation_matrix = x_optimum(3*ED_Parameter.num_nodes+1: end,:);


vertices.positions  = pointcloud;
pointcloud_new      = pointcloud;
vertices.weights    = UpdateWeights(vertices.positions,ED_Parameter.node.positions,ED_Parameter.num_nearestpts);
for i = 1 : size(vertices.positions,1)
    vertices_tmp = [0,0,0];
    for j = 1 : ED_Parameter.num_nearestpts
        vertices_pts = vertices.positions(i,:);
        weight_tmp = vertices.weights(i,2*j);
        mapped_point = Map_Points( vertices_pts, ED_Parameter.node, vertices.weights(i,2*j-1));
        vertices_tmp = vertices_tmp + weight_tmp * mapped_point;
    end
    pointcloud_new(i,:) = vertices_tmp;
end




end

