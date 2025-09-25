function [ J_x ] = JacobianF( x, ED_Parameter )
%JACOBIANF Summary of this function goes here
%   Detailed explanation goes here
num_nodes           = ED_Parameter.num_nodes;
node                = ED_Parameter.node;
control_vertices    = ED_Parameter.control_vertices;
num_nearestpts      = ED_Parameter.num_nearestpts;
vertices_prior      = ED_Parameter.vertices_prior;

num_connection         = num_nearestpts - 1; %  Connecting points excluding itself
num_transformed_points = size(control_vertices.after,1);
J_x = zeros(num_nodes * (6 + 3 * num_connection) + 3 * num_transformed_points,num_nodes * 12);

J_x_node = zeros(6 + 3 * num_connection,12);

for i = 1 : num_nodes
    x_node = x((i-1)*12+1:i*12,1);    % x_node denode variables of each node
    position_x = (i - 1) * (6 + 3 * num_connection) + 1;
    position_y = (i - 1) * 12 + 1;
    position_xx = i * (6 + 3 * num_connection);
    position_yy = i * 12;
    
    J_x_node(1,:) = [x_node(4),x_node(5),x_node(6),x_node(1),x_node(2),x_node(3),0,0,0,0,0,0];
    J_x_node(2,:) = [x_node(7),x_node(8),x_node(9),0,0,0,x_node(1),x_node(2),x_node(3),0,0,0];
    J_x_node(3,:) = [0,0,0,x_node(7),x_node(8),x_node(9),x_node(4),x_node(5),x_node(6),0,0,0];
    J_x_node(4,:) = [2*x_node(1),2*x_node(2),2*x_node(3),0,0,0,0,0,0,0,0,0];
    J_x_node(5,:) = [0,0,0,2*x_node(4),2*x_node(5),2*x_node(6),0,0,0,0,0,0];
    J_x_node(6,:) = [0,0,0,0,0,0,2*x_node(7),2*x_node(8),2*x_node(9),0,0,0];
    
    node_positions = node.positions(i,:)';
    for j = 1 : num_connection
        index_brothernode = node.weight(i,2*j+1);       
        brothernode_positions = node.positions(index_brothernode,:)';
        delta_position = brothernode_positions - node_positions;
        J_x_node(6+3*(j-1)+1:6+3*j,:) = [delta_position(1)*eye(3,3),delta_position(2)*eye(3,3),delta_position(3)*eye(3,3),0.6*eye(3,3)];  

        potision_k = index_brothernode * 12 - 2;
        J_x(position_x+6+3*(j-1):position_x+6+3*j-1,potision_k:potision_k+2) = -0.6*eye(3,3);      
    end
       
    J_x(position_x:position_xx,position_y:position_yy) = J_x_node;
end

position_x = num_nodes*(6+3*num_connection);
for i = 1 : num_transformed_points
    vertice_prior = control_vertices.prior(i,:)';
    vertice_after = control_vertices.after(i,:)';   
    for j = 1 : num_connection + 1
        node_index = vertices_prior.weight(i,2*j-1);
        node_positions = node.positions(node_index,:)';
        weight_vertice = vertices_prior.weight(i,2*j);
        temp = (vertice_prior - node_positions);
        J_x(position_x+3*(i-1)+1:position_x+3*i, (node_index-1)*12+1:node_index*12) = weight_vertice*[temp(1)*eye(3,3),temp(2)*eye(3,3),temp(3)*eye(3,3),eye(3,3)];
    end
end

end

