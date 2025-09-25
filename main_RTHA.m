close all
clear all
clear

% load data
load('./Observation/Camera_simulation.mat')
load('./Observation/Observation.mat')
% load('./Observation/Observation_resampled.mat')
load('./Observation/Images.mat')
load('./Observation/Camera.mat')

% run('./curves_align/jdtuck-fdasrvf_MATLAB-3.6.3.0/setup.m');

acetabulum_settled = stlread('./Observation/caiyun_acetabulum_setteled_v0.stl'); % CT-segmented acetabulum model

% model_general = stlread('./Observation/caiyun_fitted_acetabulum_hemisphere_dense.stl'); % general model
model_general = stlread('./Observation/noise_level_10_fitted_acetabulum_hemisphere.stl'); % general model

addpath('camera/');
addpath('utils/');
addpath('curves_align/');
addpath('curves_align/jdtuck-fdasrvf_MATLAB-3.6.3.0/')
addpath('EulerDerivation');
% addpath('data/');

model_gt.faces = acetabulum_settled.ConnectivityList;
model_gt.vertices = acetabulum_settled.Points;

model.faces = model_general.ConnectivityList;
model.vertices = model_general.Points;

% model.faces = model_gt.faces;
% model.vertices = model_gt.vertices;

% % change the overall scale of the general model
% scale_factor = 1.20;
% center_pt = mean(pointcloud_w);
% center_pts = pointcloud_w - center_pt;
% center_pts_scaled = center_pts * scale_factor;
% pts_scaled = center_pts_scaled + center_pt;

% model.faces = model_gt.faces;
% model.vertices = model_gt.vertices;



% % noise level
% euler_noise_level = 0.0;
% translation_noise_level = 0;
% 
% % first view
% R_cw = Camera{1, 1}.R;
% t_cw = Camera{1, 1}.t;
% 
% [Alpha, Beta, Gamma] = InvRotMatrixYPR22(R_cw);
% 
% % add noise to pose
% Euler_noise = euler_noise_level*(rand(3,1)-0.5);
% translation_noise = translation_noise_level*(rand(3,1)-0.5);
% 
% Alpha_noised = Alpha + Euler_noise(1);
% Beta_noised = Beta + Euler_noise(2);
% Gamma_noised = Gamma + Euler_noise(3);
% 
% Camera{1, 1}.R = RMatrixYPR22(Alpha_noised,Beta_noised,Gamma_noised); 
% Camera{1, 1}.t = t_cw + translation_noise;
% 
% % second view
% R_cw = Camera{1, 2}.R;
% t_cw = Camera{1, 2}.t;
% 
% [Alpha, Beta, Gamma] = InvRotMatrixYPR22(R_cw);
% 
% % add noise to pose
% Alpha_noised = Alpha + Euler_noise(1);
% Beta_noised = Beta + Euler_noise(2);
% Gamma_noised = Gamma + Euler_noise(3);
% 
% Camera{1, 2}.R = RMatrixYPR22(Alpha_noised,Beta_noised,Gamma_noised); 
% Camera{1, 2}.t = t_cw + translation_noise;

% reconstruct deformation
acetabulum_rst = AortaDef2D(model, model_gt, Camera, Observation, Images);
acetabulum_rst = saveToStruct(acetabulum_rst);

% check result
model_rst.vertices = acetabulum_rst.modelVertices';
model_rst.faces = model.faces;
TR = triangulation(model_gt.faces,model_gt.vertices);
normal_gt = vertexNormal(TR);

[ ~,meanError,~,error ] = computeError_result2groundtruth(model_rst.vertices, model_gt.vertices, normal_gt);

fprintf('mean error, std_error:  %f,  %f\n', mean(error), std(error));

% moving = pointCloud(model_rst.vertices);
% fixed = pointCloud(model_gt.vertices);
% [tform,movingReg] = pcregistericp(moving,fixed);
% 
% model_rst.vertices = movingReg.Location;

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

save model_rst_using_noise_5_2_SRVF.mat model_rst;
TR = triangulation(model_rst.faces, model_rst.vertices);
stlwrite(TR,'mode_reconstruction_noise_5_2_SRVF.stl','binary')

%% reconstruct error
data = error;
Q1 = prctile(data, 25);
Q3 = prctile(data, 97.5); % 75
IQR = Q3 - Q1;

lowerBound = Q1 - 0 * IQR;
upperBound = Q3 + 0 * IQR;

id = abs(error) > upperBound;
data(id) = upperBound;

error_result = data;

fprintf('mean error, std_error:  %f,  %f\n', mean(error_result), std(error_result));

figure
patch(model_rst,'FaceVertexCData',error_result,'FaceColor','interp','facealpha',0.8,'edgecolor','none');
title('\fontsize{11}Error (util: mm)')
% camorbit(gca, -10, 0, 'data', [0, 0, 1])
view(2,-2)
% view(0,270)
c = colorbar;
axis equal
box on
camlight
material('dull');
% camorbit(gca, 20, 0, 'data', [0, 1, 0])
% view([0 270]);
%xlim([40,190])
%ylim([0,300])
%xlim(Xlim{gt_id})
%zlim(Ylim{gt_id})
% xlabel('X')
% ylabel('Y')
% zlabel('Z')


%% compare original model to ground truth
[ ~,meanError_initial,~,error_initial] = computeError_result2groundtruth(model.vertices, model_gt.vertices, normal_gt);
fprintf('mean error initial, std_error_initial:  %f,  %f\n', meanError_initial, std(error_initial));

% figure
% patch(model,'facecolor',[0 0 1],'facealpha',0.6,'edgecolor','none');
% patch(model_gt,'facecolor',[0 1 0],'facealpha',0.6,'edgecolor','none');
% camlight
% legend('result','ground truth')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% axis equal


% boxplot
x1 = error_initial;
x2 = error;
x = [x1; x2];

g1 = repmat({'initial'},size(x1,1),1);
g2 = repmat({'result'},size(x2,1),1);
g = [g1; g2];

boxplot(x,g)

txt1 = {['\fontsize{10} Mean: ', sprintf( '%.*f', 3, mean(error))],...
        ['\fontsize{10} Std: ', sprintf( '%.*f', 3, std(error))]};
text(1,7,txt1)

data = error_initial;
Q1 = prctile(data, 25);
Q3 = prctile(data, 75);
IQR = Q3 - Q1;

lowerBound = Q1 - 1.5 * IQR;
upperBound = Q3 + 1.5 * IQR;

id = find(abs(error_initial) > upperBound);
error_initial(id) = upperBound;


