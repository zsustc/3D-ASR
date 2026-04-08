function [ pointcloud_new,center_pts ] = readyToPhoto( pointcloud,delta_x,delta_y,delta_z )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Script:   (i) Transform pointcloud's XY cordinate to [0,0]. 
%             (ii)Set model to the appropriate depth of front end of camera.(Z value)
%             (iii) Shift camera center
%   Method:   
%   Input:    
%             pointcloud:       3¡ÁN pointcloud
%             delta_x:          X shift;+ means shift camera center
%                               positive
%             delta_y:          Y shift;+ means shift camera center
%                               positive
%             delta_z:          delta Z for Z field
%   Returns:  
%             pointcloud_new:   Observable point cloud
%             center_xy:        Center of pointcloud
%   Author:   Jingwei Song.   02/08/2016 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
center_pts = mean(pointcloud);

pointcloud_new = pointcloud;
pointcloud_new(:,1) = pointcloud(:,1) - center_pts(1) - delta_x;
pointcloud_new(:,2) = pointcloud(:,2) - center_pts(2) - delta_y;
pointcloud_new(:,3) = pointcloud(:,3) - delta_z;

end

