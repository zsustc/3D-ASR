function [contour_update, contour, contour_index, index_contour2Vertices] = calculate_projectionContour_update(vertices, radius)
%calculate projection contour using projection vertices




shp = alphaShape(vertices(:,1),vertices(:,2), radius); % the value of obj.alphaShape 
[~,P] = boundaryFacets(shp);
boundary_index = zeros(size(P,1),1);
for ll=1:size(P,1)
    id_tem = find(vertices(:,1)==P(ll,1) & vertices(:,2)==P(ll,2));
    
    if (size(id_tem,1)==1)
        boundary_index(ll) = id_tem;   % only select the points that is not overlap
    else
        boundary_index(ll) = NaN;      % use a simple way to remove the redundant boundary
    end
end
boundary_index(isnan(boundary_index)) = [];  % remove nan

contour_index = boundary_index;
contour = vertices(contour_index,1:2);        % contour of model

%% sorting the contour points
sorted_indices = nearest_neighbor_sort(contour); %
sorted_contour = contour(sorted_indices, :); % 获取排序后的数据

%% resampling the points
N = 2000;
[sampled_contour, original_indices_contour] = curve_sampling(sorted_contour', N);

%% loop the vertices to find the sampled_contour's closest point
sampled_contour = sampled_contour';
num_sampled= size(sampled_contour, 1);
num_modelVertices = size(vertices,1);

index_contour2Vertices = zeros(0,1);  
contour_matched = zeros(0,2);

dist_threshold = 2;
for i = 1:num_sampled

    position_sampled_pt = sampled_contour(i,:);

    % calculate the distance between this observation and modelContour
    tem_dist = repmat(position_sampled_pt, num_modelVertices,1)-vertices;  

    dist_contour2Vertices = sqrt((tem_dist(:,1)).^2 + (tem_dist(:,2)).^2);    % only use the euclidean distance
    
    [dist_min, id_min] = min(dist_contour2Vertices);
    if dist_min < dist_threshold
        index_contour2Vertices = [index_contour2Vertices;id_min];
        contour_matched = [contour_matched; vertices(id_min,:)];
    end    
end

contour_update = unique(contour_matched,'rows');

end
