clc;clear all;close all;

addpath(genpath('deform_toolbox'));
addpath('pinholeModel');

%====================Observation control===================%
%   Close loop to see the model
%   PS: _w world coordinate
%       _c camera coordinate
observ_radius  = 300;           %   Observation radius (center at [0,0,0]) YZ plane
num_frame      = 5;           %   Number of loops
max_deform     = 3;            %   Maximum of key point translation(Gauss distribution)

%   Default observation direction is from camera center to model center
%   alpha beta gamma defines minor deviations
deviation_angle    = [0,0,0];  %   Deviation angle. in Rad; "Euler angle"
bool_controlpts    = 1;        %   1 means control points
%====================================================%


%   Generate depth
%====================Depth control============%
    cameraIntrinsicParam = [520     0   320
                             0    520   240
                             0     0     1];
    num_imagerow = 480;
    num_imagecol = 640;   
%====================================================%

% % Open one model
% [filename, pathName] = uigetfile('*.stl', 'Select first model');
% [fout1, vout1, cout1] = ReadSTLACSII(filename) ;
% figure(1);
% trisurf ( fout1, vout1(:,1), vout1(:,2), vout1(:,3));
% axis equal;
% xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis');

model = stlread('fitted_acetabulum_hemisphere_remesh.stl');
vout1 = model.Points;

%   Move center of mass to [0,0,0]
center_mass = mean(vout1);
vout1 = vout1 - repmat(center_mass,[size(vout1,1),1]);
center_mass = [0,0,0];

% Record control points
num_controlpts     = 30;
control_pts_matrix = zeros(num_controlpts,3*num_frame);
mask_controlpts    = 1 : ceil(size(vout1,1)/num_controlpts) : size(vout1,1);


angle        = 2 * pi / (10*num_frame);   
pointcloud_w = vout1;


v_pointcloud = zeros(size(pointcloud_w,1),1);
v_keypts     = zeros(size(pointcloud_w,1),1);
percentkeypoints = 0.005;
for i = 1 : num_frame
    
    %   Deform pointcloud and save it
    %====================Deformationo control============%
    region_percent = 0.4;      %   Percent of region to be chosen for finding max & min
    num_nearestpts = 6;        %   for calculating weights
    grid_size      = 6;       %   Downsampling grid size
    %====================================================%
    [keypts_old,keypts_new] = pickUpPoints(pointcloud_w,region_percent,max_deform);
    extra_deform_level = 15;

    % Euler_noise = euler_noise_level*(rand(3,1)-0.5);
    % translation_noise = translation_noise_level*(rand(3,1)-0.5);

    size_keypts = size(keypts_new);
    keypts_new = keypts_new + extra_deform_level * (rand(size_keypts)-0.5);
    pointcloud_w0         = pointcloud_w;

    figure
    hold on
    pcshow(pointcloud_w0)
    plot3(keypts_new(:,1),keypts_new(:,2),keypts_new(:,3),'ro')
    plot3(keypts_old(:,1),keypts_old(:,2),keypts_old(:,3),'mo')


    [pointcloud_w,ED_Parameter] = embedDeform( pointcloud_w,keypts_old,keypts_new,...
                                           num_nearestpts,grid_size);
    
    vout1 = model.Points;
    %   Move center of mass to [0,0,0]
    center_mass = mean(vout1);
    pointcloud_w = pointcloud_w + repmat(center_mass,[size(vout1,1),1]);

    save pointcloud_w_deform_level_15.mat pointcloud_w;



    %temp_saveCameraCordin;
    
    %   Camera center in world coordinate
    camera_center_w = [0,observ_radius*sin((i+10)*angle),observ_radius*cos((i+10)*angle)];  
    
    
    %======= Save control points ============%
    pointcloud_w0_c = world2CameraCord(pointcloud_w0,camera_center_w,deviation_angle);          %   world to camera cord
    depthimagetmp = points2depth(pointcloud_w0_c,fout1,num_imagerow,...                         %   camera to depth
                           num_imagecol,cameraIntrinsicParam);
    
    %   Modified by Jingwei; Creat visibility
    tmp = getVisibility(pointcloud_w0_c,num_imagerow,...                         %   camera to depth
                           num_imagecol,depthimagetmp,cameraIntrinsicParam,5);
    v_pointcloud = v_pointcloud + tmp;
    v_pointcloud(v_pointcloud>1) = 1;
    
    pointcloud_w0_c = depth2XYZcamera(cameraIntrinsicParam,depthimagetmp,num_imagerow,num_imagecol);    %   depth to camera
    pointcloud_w0 = CameraCord2world(pointcloud_w0_c,camera_center_w,deviation_angle);          %   camera cord to world
    level=randi([1,size(pointcloud_w0,1)],100,1);
    control_points.prior = pointcloud_w0(level,:);
    weights    = UpdateWeights(control_points.prior,ED_Parameter.node.positions,ED_Parameter.num_nearestpts);
for m = 1 : size(control_points.prior,1)
    vertices_tmp = [0,0,0];
    for n = 1 : ED_Parameter.num_nearestpts
        vertices_pts = control_points.prior(m,:);
        weight_tmp = weights(m,2*n);
        mapped_point = Map_Points( vertices_pts, ED_Parameter.node, weights(i,2*n-1));
        vertices_tmp = vertices_tmp + weight_tmp * mapped_point;
    end
    control_points.after(m,:) = vertices_tmp;
end
    control_points.prior = world2CameraCord(control_points.prior,camera_center_w,deviation_angle);
    control_points.after = world2CameraCord(control_points.after,camera_center_w,deviation_angle);
%     filename = ['control_points',int2str(i)];
%     save(filename,'control_points');
    %======= Save control points END ============%
    
    
    [pointcloud_rot_c,pose_matrix] = world2CameraCord(pointcloud_w,camera_center_w,deviation_angle);
    
%     filename = ['model',int2str(i),'.stl'];
%     stlwrite(filename,fout1,pointcloud_rot_c,'mode','ascii');
                                
    
    
    
    
    depthimage = points2depth(pointcloud_rot_c,fout1,num_imagerow,...
                           num_imagecol,cameraIntrinsicParam);
    datainfo_depth      = zeros(3,3);
    datainfo_depth(1,1) = num_imagerow;
    datainfo_depth(1,2) = num_imagecol;
    datainfo_depth(2,:) = camera_center_w;
    datainfo_depth(3,:) = deviation_angle;
%     filename = ['depth',int2str(i)];
%     save(filename,'depthimage');
%     filename = ['depthinfo',int2str(i)];
%     save(filename,'datainfo_deptfigure;
    %pointcloud_new = CameraCord2world(pointcloud_new,camera_center_w,deviation_angle);
    plot3(pointcloud_w(:,1),pointcloud_w(:,2),pointcloud_w(:,3),'.','MarkerSize',0.2,'Color','r');
    hold on;
    xyzPoints = depth2XYZcamera(cameraIntrinsicParam,depthimage,num_imagerow,num_imagecol);
%      filename = ['xyz_and_pose',int2str(i)];
%      save(filename,'xyzPoints','pose_matrix');
    %load depthinfo1.mat;
    
    camera_center_w = datainfo_depth(2,:);
    deviation_angle = datainfo_depth(3,:);
    xyzPoints1 = CameraCord2world(xyzPoints,camera_center_w,deviation_angle);
    
    xyzPoints1 = pointcloud_w(logical(v_pointcloud),:);
    
%     xyzPoints1(:,3) = xyzPoints1(:,3) + 0.1;
    plot3(xyzPoints1(:,1),xyzPoints1(:,2),xyzPoints1(:,3),'*','MarkerSize',1,'Color','b');
    xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis');
    title('Pin-hole camera observation');
    
    %   Visualize with camera
    r          = vrrotvec([0,0,1], -1*camera_center_w);
    R_camera   = vrrotvec2mat(r);
    R_camera   = R_camera';
    cam = plotCamera('Location',camera_center_w,'Orientation',R_camera,'Opacity',0,'Size',5);
    limit_global=[-300 300  -300 300  -300 300 ];
    axis(limit_global);
    
    %================For test===================%
    
    
    if(i==17)
    end
    %   Visualize key points
    num_keypts     = round(sum(v_pointcloud,1) * percentkeypoints);
    num_keypts_new = num_keypts - sum(v_keypts,1);
    if(num_keypts_new > 0)
        v_keyptstmp = randi([1 size(pointcloud_w,1)],1,num_keypts_new);
    end
    for m = 1:size(v_keyptstmp,2)
        ind = v_keyptstmp(m);
        for n = ind:size(v_keypts,1)
            if(v_keypts(n) == 0 & v_pointcloud(n) == 1)
                v_keypts(n) = 1;
                break;
            end
        end
    end
    xyzPoints_key = pointcloud_w(logical(v_keypts),:);
    plot3(xyzPoints_key(:,1),xyzPoints_key(:,2),xyzPoints_key(:,3),'*','MarkerSize',2,'Color','g');
    
    %   Save figure
    saveas(gcf,int2str(i),'jpg')
    
    close all;
end




