function [] = control_pts_correspondenceKNN(obj)   
%main function to calculate correspondence
% previous commit:
% use function for re-calculating correspondence among gaussian-newton iteration
% this is an extention of my previous code

% output: controlpoints: before and after; relative pose
%         indexCorrespondence_observation2ModelContour_All: corresponding index of model contour
%         indexCorrespondence_observation2Model_All:        corresponding index of model
%         observation_EDGraph_All:                          controlvertices: after
%         CorrespondModelVertices_All:                      controlvertices: before

% the correspondence threshold affects a lot, especially for the ocludded part
% Yanhao 20190314


for k=1:obj.num_frames
    %% model
    modelVertices_cam = obj.Camera{k}.R * obj.modelVertices + obj.Camera{k}.t;
    [model_projected, projected_points_index] = ...
        pointCloud_perspective_projection(modelVertices_cam, obj.Camera{k}.K, obj.Camera{k}.rows, obj.Camera{k}.cols);

    [ver_row, ver_col] = find(model_projected == 1);
    vertex_proj = [ver_col, ver_row];

    [proj_contour, proj_contour_id] = calculate_projectionContour(vertex_proj, obj.alphaShape);  % calculate projection contour
    
    % figure % configure alpha shape
    % hold on
    % plot(proj_contour(:,1), proj_contour(:,2),'r.')
    % plot(vertex_proj(:,1), vertex_proj(:,2),'b.')

    %% observation and normal vector
    obser = obj.Observation{k}.obs_p;
    
    %% correspondence
    %% correspondence
    [index_obser2ModelContour, observation_after] = ...
    calculate_correspondence_observation2ModelContourKNN(obser, proj_contour, obj.dist_threshold);

    indexCorrespondence_observation2Model = proj_contour_id(index_obser2ModelContour,1); % the index of 3D model vertices

    %% save output
    proj_vers_index = vertex_proj(indexCorrespondence_observation2Model,:); % col, row

    obs_vertices_pixel = zeros(0,2);
    obs_vertices_model_cam = zeros(0,3);
    obs_vertices_model = zeros(0,3);
    
    for i = 1:size(proj_vers_index,1)
        u_col = proj_vers_index(i,1);
        v_row = proj_vers_index(i,2);
        index_tmpt = projected_points_index(v_row, u_col);
        if index_tmpt > 0
            obs_vertices_pixel = [obs_vertices_pixel; u_col, v_row]; % u,v
            obs_vertices_model_cam = [obs_vertices_model_cam; modelVertices_cam(:,index_tmpt)'];
            obs_vertices_model = [obs_vertices_model; obj.modelVertices(:,index_tmpt)'];
        end
    end

    % control vertices
    obj.control_vertex_prior{k} = obs_vertices_model';  % 3D position of observed vertices beform deformation, they are in model frame (before rotation)
    obj.control_vertex_after{k} = observation_after';                                          % observation used as control after 2D pixel coordinate
     
    figure
    %imshow(model_projected)
    hold on
    plot(observation_after(:,1), observation_after(:,2), 'r.')
    plot(obs_vertices_pixel(:,1), obs_vertices_pixel(:,2), 'b.')
    plot(proj_contour(:,1), proj_contour(:,2), 'm.')
    plot([observation_after(:,1)'; obs_vertices_pixel(:,1)'], [observation_after(:,2)'; obs_vertices_pixel(:,2)'], 'g-')

    %% corresponding ed nodes
    [obj.control_vertex_prior_weight_id{k}, obj.control_vertex_prior_weight{k} ] = updateWeight_knn(obj.control_vertex_prior{k}', obj.node_position', obj.num_nearestpts); 

    % other output
    obj.num_controlVertices(k) = size(obj.control_vertex_after{k},2);     % used in jacobian and F
end
obj.num_control_pts_all = sum(obj.num_controlVertices);         % all transformed points used for construct jacobian and F
end