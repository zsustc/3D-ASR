function [proj] = camera_perspective_projection_model(v, K, R, t)
% R is rotation matrix
% t is the pose translation
% K is the camera intrinsic parameter
% perspective projection model
proj = K*(R*v+t);
proj = proj(1:2,:) ./ proj(3,:); % [u, v, 1]

end