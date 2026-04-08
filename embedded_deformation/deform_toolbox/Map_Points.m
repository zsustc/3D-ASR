function [ mapped_vertices ] = Map_Points( vertice, node, node_index )
%MAP_POINTS Summary of this function goes here
%   map a point according to a given node
node_positions = node.positions(node_index,:);
node_rotation_matrix = node.rotation_matrix(3*node_index-2:3*node_index,:);
node_transformation_matrix = node.transformation_matrix(node_index,:);

mapped_vertices = node_rotation_matrix * (vertice' - node_positions') + node_positions' + node_transformation_matrix';
mapped_vertices = mapped_vertices';
end

