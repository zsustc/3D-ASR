function [aligned_obs, aligned_model_contour, Geod] = ...
    calculate_correspondence_observation2ModelContourSRVF(observation, modelContour)
% X2n,X1 are from geodesic 
% X2n=Geod(:,:,end), X1=Geod(:,:,1);
% figure;hold on
% plot(observation(:,1), observation(:,2),'r.')
% plot(modelContour(:,1), modelContour(:,2),'b.')

%% observation 已经提前预处理：sort，sampling, N = 2000.避免每次循环都处理。 observation是固定不变的
%% 不需要每次循环都处理一次
% % sort the points along the curve, 设置最靠边的边为第一个点,对于其他线段，可能需要调整
% sorted_indices_obs = nearest_neighbor_sort(observation); %
% sorted_obs = observation(sorted_indices_obs, :); % 获取排序后的数据
% 
% % figure
% % hold on
% % plot(observation(:,1), observation(:,2),'r.')
% % for i = 1:size(sorted_obs, 1)
% %     plot(sorted_obs(i,1), sorted_obs(i,2),'bo')
% %     pause(0.01)
% % end

sorted_indices_m_contour = nearest_neighbor_sort(modelContour);
sorted_m_contour = modelContour(sorted_indices_m_contour, :); % 获取排序后的数据

% [sorted_m_contour, sorted_indices_m] = nearest_neighbor_sort_ccw(modelContour, true);
% [~, sorted_indices] = sort(angles, 'descend');
% [sorted_observation, sorted_indices_obs] = nearest_neighbor_sort_ccw(observation, true);
% 
% figure
% hold on
% plot(modelContour(:,1), modelContour(:,2),'r.')
% for i = 1:size(sorted_m_contour, 1)
%     plot(sorted_m_contour(i,1), sorted_m_contour(i,2),'bo')
%     pause(0.01)
% end

direction_m_contour = checkCurveDirection(sorted_m_contour);
direction_obs_contour = checkCurveDirection(observation);

if direction_m_contour ~= direction_obs_contour
    sorted_m_contour = flipud(sorted_m_contour); 
end

% detectOutliers from sorted_m_contour
epsilon = 50; % 调整半径参数
minPts = 10; % 最小邻居点数
[mainPoints, outlierPoints] = detectOutliersDBSCAN(sorted_m_contour, epsilon, minPts);

% % 可视化结果
% figure; hold on;
% scatter(sorted_m_contour(:,1), sorted_m_contour(:,2), 10, 'k', 'filled', 'DisplayName', 'original point');
% scatter(mainPoints(:,1), mainPoints(:,2), 30, 'b', 'filled', 'DisplayName', 'main curve');
% scatter(outlierPoints(:,1), outlierPoints(:,2), 30, 'r', 'filled', 'DisplayName', 'Outliers');
% legend;
% title('detectOutliersDBSCAN');
% axis equal;

sorted_m_contour = mainPoints;

% resample the curves
% num_points = size(sorted_obs, 1); % the number of sampled points depend on the number of obs
% sorted_obs_sampled = sorted_obs';

num_points = 2000; % 2000
option.plot_geod = false;
option.plot_reg = false; % false
option.plot_reparam = false;
option.print = false;
option.N = num_points;
option.stp = 6; % 6, 25
option.closed = false; % true for closed curve

X1 = observation';
X2 = sorted_m_contour';

% X1 = sorted_m_contour';
% X2 = sorted_obs';

[dist,X2n,q2n,X1,q1,matching_indices_X1_orig,matching_indices_X2_orig,Geod] = ...
    pairwise_align_curves(X1, X2, option);

% aligned points
aligned_obs = observation(matching_indices_X1_orig,:);
aligned_model_contour = sorted_m_contour(matching_indices_X2_orig,:);

end