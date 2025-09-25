function [ f_x ] = CalculateF( x ,ED_Parameter )
%UNTITLED Summary of this function goes here
%   Calculating the vector value node by node
num_nodes           = ED_Parameter.num_nodes;
node                = ED_Parameter.node;
control_vertices    = ED_Parameter.control_vertices;
num_nearestpts      = ED_Parameter.num_nearestpts;
vertices_prior      = ED_Parameter.vertices_prior;

num_connection         = num_nearestpts - 1; %  Connecting points excluding itself
num_transformed_points = size(control_vertices.after,1);
f_x = zeros(num_nodes * (6 + 3 * num_connection) + 3 * num_transformed_points,1);

% f_x_node is the temperary f_x for each node
f_x_node = zeros(6 + 3 * num_connection,1);
for i = 1 : num_nodes
    x_node = x((i-1)*12+1:i*12,1);    % x_node denode variables of each node 
    
    f_x_node(1) = x_node(1) * x_node(4) + x_node(2) * x_node(5) + x_node(3) * x_node(6);
    f_x_node(2) = x_node(1) * x_node(7) + x_node(2) * x_node(8) + x_node(3) * x_node(9);
    f_x_node(3) = x_node(4) * x_node(7) + x_node(5) * x_node(8) + x_node(6) * x_node(9);
    f_x_node(4) = x_node(1)^2 + x_node(2)^2 + x_node(3)^2 - 1;
    f_x_node(5) = x_node(4)^2 + x_node(5)^2 + x_node(6)^2 - 1;
    f_x_node(6) = x_node(7)^2 + x_node(8)^2 + x_node(9)^2 - 1;

    node_rotation_matrix = [x_node(1),x_node(4),x_node(7);
                            x_node(2),x_node(5),x_node(8);
                            x_node(3),x_node(6),x_node(9)];
    node_positions = node.positions(i,:)';
    node_transformation_matrix = [x_node(10),x_node(11),x_node(12)]';
    for j = 1 : num_connection
        index_brothernode = node.weight(i,2*j+1);       
        brothernode = x((index_brothernode-1)*12+1:index_brothernode*12,1);
        brothernode_positions = node.positions(index_brothernode,:)';
        vertice_transform = [brothernode(10),brothernode(11),brothernode(12)]';
        f_x_node(6+3*(j-1)+1:6+3*j) = node_rotation_matrix * (brothernode_positions - node_positions) + node_positions + node_transformation_matrix - (brothernode_positions + vertice_transform);
    end
        
    f_x((6+3*num_connection)*(i-1)+1:(6+3*num_connection)*i) = f_x_node;
    f_x_node = zeros(6 + 3 * num_connection,1);    
end


position_x = num_nodes*(6+3*num_connection);
for i = 1 : num_transformed_points  
    point_rot_trans = [0, 0, 0]';
    vertice_prior = control_vertices.prior(i,:)';
    vertice_after = control_vertices.after(i,:)';    
    for j = 1 : num_connection + 1      
        node_index = vertices_prior.weight(i,2*j-1);
        weight_vertice = vertices_prior.weight(i,2*j);
        node_positions = node.positions(node_index,:)';
        x_node = x((node_index-1)*12+1:node_index*12,1);    % x_node denode variables of each node 
        node_rotation_matrix = [x_node(1),x_node(4),x_node(7);
                                x_node(2),x_node(5),x_node(8);
                                x_node(3),x_node(6),x_node(9)];
        node_transformation_matrix = [x_node(10),x_node(11),x_node(12)]';
        point_rot_trans = point_rot_trans + weight_vertice * (node_rotation_matrix * (vertice_prior - node_positions) + node_positions + node_transformation_matrix);     
    end
    f_x(position_x+3*(i-1)+1:position_x+3*i,1) = point_rot_trans - vertice_after;
end

end

