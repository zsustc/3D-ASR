function [ fmin ] = Optimization_ObjectFunc( x0 )
%OPTIMIZATION_OBJECTFUNC Summary of this function goes here
%   Detailed explanation goes here
global control_vertices;
global vertices_prior;
global num_mapping_node;
global num_nearestpts;
global node_storage;
global num_fixed_nodes;
global num_free_nodes;

node_fixedpositions = node_storage.fixedpositions;
node_fixedrotation_matrix = node_storage.fixedrotation_matrix;
node_fixedtranslation = node_storage.fixedtransformation_matrix;

node_freepositions = node_storage.freepositions;
node_freerotation_matrix = x0(1: 3*num_free_nodes,:);
node_freetranslation = x0(3*num_free_nodes + 1: end,:);

%node_tmp.positions = node_storage.positions;
node_tmp.positions = [node_fixedpositions;node_freepositions];
node_tmp.rotation_matrix = [node_fixedrotation_matrix;node_freerotation_matrix];
node_tmp.transformation_matrix = [node_fixedtranslation;node_freetranslation];

num_free_node = size(node_freepositions,1);

% 1st  Constraint -- rotation
error_rotation = 0;
for i = 1 : num_free_node
    rotation_tmp = node_freerotation_matrix(3*i-2:3*i,:);
    column1 = rotation_tmp(:,1);
    column2 = rotation_tmp(:,2);
    column3 = rotation_tmp(:,3);
    error_rotation = error_rotation + (column1'*column2)^2 + (column1'*column3)^2 + (column2'*column3)^2 + (column1'*column1 - 1)^2 + (column2'*column2 - 1)^2 + (column3'*column3 - 1)^2;
end

% 2st  Constraint -- regularization

% weights_nodes = zeros(num_mapping_node,num_mapping_node);    %weight matrix.
% for i = 1 : num_mapping_node
%     temp_colunm_weight = 0;
%     for j = 1 : num_mapping_node
%        weights_nodes(i,j) = sqrt((node_position(i,1) - node_position(j,1))^2 + (node_position(i,2) - node_position(j,2))^2 + (node_position(i,3) - node_position(j,3))^2);
%        temp_colunm_weight = temp_colunm_weight + weights_nodes(i,j);
%     end
%     %normalize weights
%     for j = 1 : num_mapping_node
%        weights_nodes(i,j) = weights_nodes(i,j) / temp_colunm_weight;
%     end
% end
% error_regulation = 0;
% for j = 1 : num_mapping_node
%     for k = 1 : num_mapping_node
%         distanceerror = Map_Points( node_tmp.positions(k,:), node_tmp, j ) - Map_Points( node_tmp.positions(k,:), node_tmp, k );
%         error_regulation = error_regulation + norm(distanceerror,2);
%     end
% end
error_regulation = 0;
if(node_storage.connectivity(1,1) ~= 0)
    for j = 1 : num_mapping_node
        num_connect_points = size(node_storage.connectivity,2) - 1;    %how many points is in an edge with this point
        for i = 1 : num_connect_points
            k = node_storage.connectivity(j,i+1);
            distanceerror = Map_Points( node_tmp.positions(k,:), node_tmp, j ) - Map_Points( node_tmp.positions(k,:), node_tmp, k );
            error_regulation = error_regulation + norm(distanceerror,2)^2;
        end
    end
else
    for j = 1 : num_mapping_node
        for k = 1 : num_mapping_node
            distanceerror = Map_Points( node_tmp.positions(k,:), node_tmp, j ) - Map_Points( node_tmp.positions(k,:), node_tmp, k );
            error_regulation = error_regulation + norm(distanceerror,2)^2;
        end
    end
end


% 3st  Constraint -- user controlled constrains
error_user_control_contraints = 0;

num_user_edit_points = size(vertices_prior.positions,1);
for i = 1 : num_user_edit_points
    point_rot_trans = [0, 0, 0];
    for j = 1 : num_nearestpts
        point_rot_trans = point_rot_trans + vertices_prior.weights(i,2*j) * Map_Points( vertices_prior.positions(i,:), node_tmp, vertices_prior.weights(i,2*j-1) );
    end
    distanceerror = point_rot_trans - control_vertices.after(i,:);
    if(norm(distanceerror,2)>0.1)
    end
    error_user_control_contraints = error_user_control_contraints + norm(distanceerror,2)^2;
end

%sum all the errors together
fmin = 1 * error_rotation + 10 * error_regulation + 100 * error_user_control_contraints;

end

