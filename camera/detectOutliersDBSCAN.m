function [mainPoints, outlierPoints] = detectOutliersDBSCAN(curve, epsilon, minPts)
    % 检测曲线中的脱离点，基于 DBSCAN 聚类
    % curve: Nx2 矩阵，表示 2D 曲线点
    % epsilon: DBSCAN 搜索半径
    % minPts: 聚类所需最小邻居点数
    
    % 执行 DBSCAN 聚类
    labels = dbscan(curve, epsilon, minPts);

    % 主体聚类: 选择最大聚类作为主体
    mainCluster = mode(labels(labels > 0)); % 最常见的非噪声标签
    mainIndices = (labels == mainCluster);

    % 分离主线和偏离点
    mainPoints = curve(mainIndices, :);
    outlierPoints = curve(~mainIndices, :);
end


