function [ visibility ] = getVisibility( pts,n_row,n_col,depth,K,threshold)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Script:   Transform a pointcloudcloud [X,Y,Z] to depth image [u,v,value]
%   Method:
%   Input:
%             K:                Camera intrinsic matrix
%             pointcloud:            1¡Á3 pointcloud
%   Returns:
%             u:                the 'u'th row of image
%             v:                the 'v'th column of image
%             value:            value of depth image [u,v]
%   Author:   Jingwei Song.     02/08/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
num_point = size(pts,1);
visibility= zeros(num_point,1);


for i = 1 : num_point
    v      = round(K(1,1)*(pts(i,1)./pts(i,3)) + K(1,3));
    u      = round(K(2,2)*(pts(i,2)./pts(i,3)) + K(2,3));
    if(u>0 & u<n_row & v>0 & v<n_col)
        if(abs(pts(i,3)-depth(u,v) < threshold))
            visibility(i,1) = 1;
        end
    end
end
end

