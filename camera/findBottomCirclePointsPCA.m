function [bottomCircleIdx, bottomCircleCenter, estimatedRadius] = findBottomCirclePointsPCA(Points, dist_threshold, radiusThreshold)

    x = Points(:,1);
    y = Points(:,2);
    z = Points(:,3);

    % Step 1: Perform PCA to get the principal components
    [coeff, score, ~] = pca([x, y, z]);  % 'score' contains the transformed points
    
    % Step 2: Find the points that are farthest along the negative third principal component (bottom region)
    % The third component in 'coeff' represents the direction of the hemisphere's normal (top direction)
    thirdComponent = score(:,3);  % The third column of 'score' represents displacement along the normal
    
    % Set a threshold to identify the bottom region (points farthest along the negative third component)
    threshold = min(thirdComponent) + dist_threshold * (max(thirdComponent) - min(thirdComponent));  % Adjust as needed
    bottomRegionPoints = thirdComponent < threshold;  % Select points near the bottom
    
    % Step 3: Estimate the center of the bottom circle
    bottomX = x(bottomRegionPoints);
    bottomY = y(bottomRegionPoints);
    bottomZ = z(bottomRegionPoints);
    bottomCircleCenter = [mean(bottomX), mean(bottomY), mean(bottomZ)];
    
    % Step 4: Estimate the radius as the distance from the center to the bottom region points
    distances = sqrt((bottomX - bottomCircleCenter(1)).^2 + (bottomY - bottomCircleCenter(2)).^2 + (bottomZ - bottomCircleCenter(3)).^2);
    estimatedRadius = mean(distances);  % Use the mean distance as the radius
    
    % Step 5: Find points that are near this bottom circle
    distFromCenter = sqrt((x - bottomCircleCenter(1)).^2 + (y - bottomCircleCenter(2)).^2 + (z - bottomCircleCenter(3)).^2);
    
    % Set a threshold for distance from the estimated radius
    % radiusThreshold = 1;  % Adjust as needed
    bottomCircleIdx = find(abs(distFromCenter - estimatedRadius) < radiusThreshold & bottomRegionPoints);
    
    % % Visualize the points
    % scatter3(x, y, z, 'b.');  % Plot the full point cloud
    % hold on;
    % scatter3(x(bottomCircleIdx), y(bottomCircleIdx), z(bottomCircleIdx), 'r.', 'LineWidth', 2);  % Plot the top circle points
    % title('Top Circle Points of the Hemisphere (PCA Method)');
    % legend('Full Point Cloud', 'Top Circle Points');
    % xlabel('X');
    % ylabel('Y');
    % zlabel('Z');

end