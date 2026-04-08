close all
clear all
clear

% load data
load('./three_views_obs/Camera.mat')
load('./data/pointcloud_w_deform_level_5.mat')
load('./data/model_gt.mat')
load('./three_views_obs/Observation.mat')

addpath('camera/');
addpath('utils/');
% addpath('data/');

% Camera{1, 1}.t = Camera{1, 1}.t + [-5,0,0]'; % x, y, z

model.faces = model_gt.faces;
model.vertices = model_gt.vertices;
% model.vertices = pointcloud_w;

% reconstruct deformation
acetabulum_rst = AortaDef2D(model, Camera, Observation);
acetabulum_rst = saveToStruct(acetabulum_rst);


% check result
model_rst.vertices = acetabulum_rst.modelVertices';
model_rst.faces = model.faces;
TR = triangulation(model_gt.faces,model_gt.vertices);
normal_gt = vertexNormal(TR);

[ ~,meanError,~,error ] = computeError_result2groundtruth( model_rst.vertices, model_gt.vertices, normal_gt);


figure
patch(model_rst,'facecolor',[0 0 1],'facealpha',0.6,'edgecolor','none');
patch(model_gt,'facecolor',[0 1 0],'facealpha',0.6,'edgecolor','none');
% patch(model,'facecolor',[1 0 0],'facealpha',0.6,'edgecolor','none');
camlight
legend('result','ground truth')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal

save model_rst.mat model_rst;

%% compare original model to ground truth
[ ~,meanError,~,error ] = computeError_result2groundtruth( model.vertices, model_gt.vertices, normal_gt);

figure
patch(model,'facecolor',[0 0 1],'facealpha',0.6,'edgecolor','none');
patch(model_gt,'facecolor',[0 1 0],'facealpha',0.6,'edgecolor','none');
camlight
legend('result','ground truth')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal


