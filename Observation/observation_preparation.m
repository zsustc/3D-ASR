close all
clear all
clc

addpath('./camera');
hemisphere_mesh = stlread('fitted_acetabulum_hemisphere_remesh.stl');

load CameraParametersGeometryAP.mat;
load CameraParametersGeometryLAT.mat;

Camera = cell(1,2);
Camera{1,1}.R = CameraParametersAP.R_cw;
Camera{1,1}.t = CameraParametersAP.t_cw;
Camera{1,1}.R = CameraParametersAP.R_cw;
Camera{1,1}.K = CameraParametersAP.K;
Camera{1,1}.cols = CameraParametersAP.cols;
Camera{1,1}.rows = CameraParametersAP.rows;

Camera{1,2}.R = CameraParametersGeometryLAT.R_cw;
Camera{1,2}.t = CameraParametersGeometryLAT.t_cw;
Camera{1,2}.R = CameraParametersGeometryLAT.R_cw;
Camera{1,2}.K = CameraParametersGeometryLAT.K;
Camera{1,2}.cols = CameraParametersGeometryLAT.cols;
Camera{1,2}.rows = CameraParametersGeometryLAT.rows;

%% obtain the observation for the first view (AP)
proj_ap = camera_perspective_projection_model(hemisphere_mesh.Points', ...
    Camera{1,1}.K, Camera{1,1}.R, Camera{1,1}.t);

points_camera_space = Camera{1,1}.R * hemisphere_mesh.Points' + Camera{1,1}.t;

[projected_image_ap, projected_points_index] = ...
    pointCloud_perspective_projection(points_camera_space, Camera{1,1}.K, ...
    Camera{1,1}.rows, Camera{1,1}.cols);

radius = 250;
[contour_ap, contour_index] = calculate_projectionContour(proj_ap', radius);

figure
imshow(projected_image_ap)
hold on
plot(contour_ap(:,1),contour_ap(:,2),'r.');
axis equal

lineStep = 8;
N_ap = LineNormals2D(contour_ap,lineStep);

Vertices = contour_ap;
N = N_ap;
figure,
imshow(projected_image_ap)
hold on
plot([Vertices(:,1) Vertices(:,1)+10*N(:,1)]',[Vertices(:,2) Vertices(:,2)+10*N(:,2)]');
plot(contour_ap(:,1),contour_ap(:,2),'r.');

%% obtain the observation for the second view (lat)
proj_lat = camera_perspective_projection_model(hemisphere_mesh.Points', ...
    Camera{1,2}.K, Camera{1,2}.R, Camera{1,2}.t);

points_camera_space = Camera{1,2}.R * hemisphere_mesh.Points' + Camera{1,2}.t;

[projected_image_lat, projected_points_index] = ...
    pointCloud_perspective_projection(points_camera_space, Camera{1,2}.K, ...
    Camera{1,2}.rows, Camera{1,2}.cols);

[contour_lat, contour_index] = calculate_projectionContour(proj_lat', radius);

figure
imshow(projected_image_lat)
hold on
plot(contour_lat(:,1), contour_lat(:,2),'r.');
axis equal

N_lat = LineNormals2D(contour_lat,lineStep);
Vertices = contour_lat;
N = N_lat;
figure,
imshow(projected_image_lat)
hold on
plot([Vertices(:,1) Vertices(:,1)+10*N(:,1)]',[Vertices(:,2) Vertices(:,2)+10*N(:,2)]');
plot(contour_lat(:,1),contour_lat(:,2),'r.');

%% save observation
Observation = cell(1,2);
Observation{1, 1}.obs_p = contour_ap;
Observation{1, 1}.obs_n = N_ap;

Observation{1, 2}.obs_p = contour_lat;
Observation{1, 2}.obs_n = N_lat;


save Observation.mat Observation;

model.vertices = hemisphere_mesh.Points;
model.faces = hemisphere_mesh.ConnectivityList;
save model.mat model;

Images = cell(1,2);
Images{1,1} = projected_image_ap;
Images{1,2} = projected_image_lat;

save Images.mat Images;

save Camera.mat Camera;

%% another view observation
% construct rotation matrix
Alpha = (0/180)*pi; % z, radians
Beta = (0/180)*pi; % y
Gamma = (-90/180)*pi; % x

% convert to rotation as order of Alpha, Beta and Gamma
R = RMatrixYPR22(Alpha,Beta,Gamma); 
