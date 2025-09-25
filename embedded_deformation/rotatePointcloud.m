function [ pointcloud_rot ] = rotatePointcloud( pointcloud,center_mass,angle)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Rotate pointcloud
%   Method:   Rotate the pointcloud w.r.t center_mass and X axis
%   Input:    pointcloud:       Pointcloud
%             center_mass:      3¡Á1 Rotation center of mass
%             angle:            Rotation angle (X axis based)
%   Returns:  pointcloud_rot:   Rotated point cloud 
%   Author:   Jingwei Song.     28/07/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rotationX =      [  1,      0,            0;
                    0, cos(angle)    -sin(angle);
                    0, sin(angle)     cos(angle);];     % X axis euler angle
                
num_point       = size(pointcloud,1);
pointcloud_rot  = zeros(num_point,3);
for i = 1 : num_point
    newpoint = rotationX * (pointcloud(i,:)' - center_mass') + center_mass';
    pointcloud_rot(i,:) =  newpoint';
end

end

