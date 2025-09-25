function [index_obser2Contour, observation_matched] = ...
    calculate_correspondence_observation2ModelContourKNN(observation, modelContour, dist_threshold)

num_observation = size(observation,1);
num_modelContour = size(modelContour,1);

% the index of model contour associated by observation.
index_obser2Contour = zeros(0,1);  
observation_matched = zeros(0,2);

for i=1:num_observation
    position_observationI = observation(i,:);
    
    % calculate the distance between this observation and modelContour
    tem_dist = repmat(position_observationI,num_modelContour,1)-modelContour;                         % same as repmat(A(a,:),m,1)-B;
    dist_observation2ModelContour = sqrt((tem_dist(:,1)).^2 + (tem_dist(:,2)).^2);    % only use the euclidean distance
    
    [dist_min, id_min] = min(dist_observation2ModelContour);
    if dist_min < dist_threshold
        index_obser2Contour = [index_obser2Contour;id_min];
        observation_matched = [observation_matched; position_observationI];
    end    

end