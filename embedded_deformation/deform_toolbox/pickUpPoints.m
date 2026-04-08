function [ keypts_old,keypts_new ] = pickUpPoints( pointcloud,region_percent,max_deform )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Pick up points
%   Method:   Pick up max&min points of a specified region(centered at
%             point center
%   Input:    pointcloud:       Current point cloud
%             region_percent:   Percent of the region of that point cloud
%             max_deform:     	Randomly chosen deform translation
%   Returns:   
%             keypts_old:       Old key points
%             keypts_new:   	New key points
%   Author:   Jingwei Song.     27/07/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

keypts_old = zeros(6,3);        %   6 points

%   Point cloud bound
max_x = max(pointcloud(:,1));
max_y = max(pointcloud(:,2));
max_z = max(pointcloud(:,3));
min_x = min(pointcloud(:,1));
min_y = min(pointcloud(:,2));
min_z = min(pointcloud(:,3));

%   Region bound
delta_x = max_x - min_x;
delta_y = max_y - min_y;
delta_z = max_z - min_z;
precent_tmp = (1 - region_percent) / 2;
max_region_x = max_x - delta_x * precent_tmp;
max_region_y = max_y - delta_y * precent_tmp;
max_region_z = max_z - delta_z * precent_tmp;
min_region_x = min_x + delta_x * precent_tmp;
min_region_y = min_y + delta_y * precent_tmp;
min_region_z = min_z - delta_z * precent_tmp;

%   Group 1. Points within region_x and region_y
index_tmp = find((pointcloud(:,1)>=min_region_x)&...
                 (pointcloud(:,1)<=max_region_x)&...
                 (pointcloud(:,2)>=min_region_y)&...
                 (pointcloud(:,2)<=max_region_y));
group1    = pointcloud(index_tmp,:);
[temp,index]   = max(group1(:,3));
keypts_old(1,:) = group1(index,:);
[temp,index]   = min(group1(:,3));
keypts_old(2,:) = group1(index,:);

%   Group 2. Points within region_x and region_z
index_tmp = find((pointcloud(:,1)>=min_region_x)&...
                 (pointcloud(:,1)<=max_region_x)&...
                 (pointcloud(:,3)>=min_region_z)&...
                 (pointcloud(:,2)<=max_region_z));
group2    = pointcloud(index_tmp,:);
[temp,index]   = max(group2(:,2));
keypts_old(3,:) = group2(index,:);
[temp,index]   = min(group2(:,2));
keypts_old(4,:) = group2(index,:);

%   Group 3. Points within region_y and region_z
index_tmp = find((pointcloud(:,2)>=min_region_y)&...
                 (pointcloud(:,2)<=max_region_y)&...
                 (pointcloud(:,3)>=min_region_z)&...
                 (pointcloud(:,3)<=max_region_z));
group3    = pointcloud(index_tmp,:);
[temp,index]   = max(group3(:,1));
keypts_old(5,:) = group3(index,:);
[temp,index]   = min(group3(:,1));
keypts_old(6,:) = group3(index,:);

%   Translate min_z in group one
keypts_new      = keypts_old;
deformpara      = randn;
while(abs(deformpara)-max_deform>0)
    deformpara      = max_deform*randn;
end
keypts_new(1,3) = keypts_new(1,3) + abs(deformpara); %   Gauss noise
end

