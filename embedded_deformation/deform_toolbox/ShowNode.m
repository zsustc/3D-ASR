function [ output_args ] = ShowNode( node,ED_Parameter )
%SHOWNODE Summary of this function goes here
%   Detailed explanation goes here
num_nodes           = ED_Parameter.num_nodes;
num_nearestpts      = ED_Parameter.num_nearestpts;


xStep = 1:1:num_nodes;
plot3(node.positions(xStep,1),node.positions(xStep,2),node.positions(xStep,3),'.','MarkerSize',10,'Color','g');
hold on;

for i = 1 : num_nodes
    pt1 = node.positions(i,:);
    for j = 1 : num_nearestpts
        k = node.weight(i,2*j-1);
        pt2 = node.positions(k,:);
        line = [pt1;pt2];
        plot3(line(:,1),line(:,2),line(:,3),'LineWidth',2,'Color','r');
        hold on;
    end    
end

axis equal;
xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis');

end

