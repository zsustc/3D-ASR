close all
clear all
clc

addpath('./EulerDerivation');
addpath('./camera');
hemisphere_mesh = stlread('fitted_acetabulum_hemisphere_remesh.stl');

load Camera.mat;
load Observation.mat;
load Images.mat;

points_camera_space_v1 = Camera{1,1}.R * hemisphere_mesh.Points' + Camera{1,1}.t;

% points_camera_space_v2 = Camera{1,2}.R * hemisphere_mesh.Points' + Camera{1,2}.t;

figure
pcshow(points_camera_space_v1')
xlabel('X')
ylabel('Y')
zlabel('Z')

% YX plane
view(0,90)
camorbit(100,0,'data',[0 1 0])
camorbit(20,0,'data',[0 0 1])
%% obtain the observation for the third view using ap intrinsic parameters
[Alpha,Beta,Gamma] = InvRotMatrixYPR22(Camera{1, 1}.R);

Alpha = Alpha + (20/180)*pi; % z, radians
Beta = Beta + (-100/180)*pi; % y
Gamma = Gamma + (0/180)*pi; % x 0

tx = Camera{1, 1}.t(1) - 1060;
ty = Camera{1, 1}.t(2) + 0;
tz = Camera{1, 1}.t(3) + 1000;

t = [tx, ty, tz]';

% convert to rotation as order of Alpha, Beta and Gamma
R = RMatrixYPR22(Alpha,Beta,Gamma); 

k = 1;
alphaShape = 5;
modelVertices = hemisphere_mesh.Points';

modelVertices_cam = R * modelVertices + t;
[model_projected, projected_points_index] = ...
    pointCloud_perspective_projection(modelVertices_cam, Camera{k}.K, Camera{k}.rows, Camera{k}.cols);

figure
pcshow(modelVertices_cam')
xlabel('X')
ylabel('Y')
zlabel('Z')


[ver_row, ver_col] = find(model_projected == 1);
vertex_proj = [ver_col, ver_row];

[proj_contour, proj_contour_id] = calculate_projectionContour(vertex_proj, alphaShape);  % calculate projection contour

figure
imshow(model_projected)
hold on
plot(proj_contour(:,1), proj_contour(:,2),'r.')
axis equal
axis on
xlabel('u')
ylabel('v')

lineStep = 8;
N = LineNormals2D(proj_contour, lineStep);
Vertices = proj_contour;

figure,
imshow(model_projected)
hold on
plot([Vertices(:,1) Vertices(:,1)+10*N(:,1)]',[Vertices(:,2) Vertices(:,2)+10*N(:,2)]');
plot(proj_contour(:,1),proj_contour(:,2),'r.');


%% save the third view result
Camera3rdviews = Camera{1,1}; % use the same intrinsic parameters as the ap view
Camera3rdviews.R = R;
Camera3rdviews.t = t;
Camera{1, end+1} = Camera3rdviews;

Observation3rdview.obs_p = proj_contour;
Observation3rdview.obs_n = N;
Observation{1, end+1} = Observation3rdview;

Images{1, end+1} = model_projected;

save Camera.mat Camera;
save Observation.mat Observation;
save Images.mat Images;



