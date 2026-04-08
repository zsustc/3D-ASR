close all
clear all
clc


load model_rst.mat;
% load data
load('./data/Camera.mat')
load('./data/model_gt.mat')

%% plot camera together with reconstruction result
initialRotation = Camera{1, 1}.R;
initialTranslation = Camera{1, 1}.t;
pose = rigidtform3d(initialRotation,initialTranslation);

rst_vertices = Camera{1, 1}.R * model_rst.vertices' + Camera{1, 1}.t;
gt_vertices = Camera{1, 1}.R * model_gt.vertices' + Camera{1, 1}.t;

model_gt.vertices = gt_vertices';
model_rst.vertices = rst_vertices';

figure
hold on
patch(model_rst,'facecolor',[0 0 1],'facealpha',0.6,'edgecolor','none');
patch(model_gt,'facecolor',[0 1 0],'facealpha',0.6,'edgecolor','none');
%cam = plotCamera(AbsolutePose=pose,Size=20);
camlight
legend('result','ground truth')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal