function [best_match1, best_match2] = surf_match(pc1, pc2, filt1, filt2, frame)
    img1 = img2d(pc1.Color);
    img2 = img2d(pc2.Color);
    
    points1 = detectSURFFeatures(rgb2gray(img1),'MetricThreshold', 30);
    points2 = detectSURFFeatures(rgb2gray(img2),'MetricThreshold', 30);
    
    [f1,vpts1] = extractFeatures(rgb2gray(img1),points1);
    [f2,vpts2] = extractFeatures(rgb2gray(img2),points2);
    
    indexPairs = matchFeatures(f1,f2) ;
    matchedPoints1 = vpts1(indexPairs(:,1));
    matchedPoints2 = vpts2(indexPairs(:,2));
    
    % package matched features, shape M*(x1,y1,x2,y2)
    data = [matchedPoints1.Location, matchedPoints2.Location];
    
%             draw_sift(img1,img2,...
%                 data(:,1:2),data(:,3:4),...
%                 data(:,1:2),data(:,3:4));
%     
%     pause
    % remove repeat and 3D-NaN points
    data = clean_match_results(data, filt1, filt2);
    
    % RANSAC to remove outliers
    [inlierIdx, outlierIdx] = ransac_pairs(data, frame);
    
    
    % draw the matching plot
    draw_sift(img1,img2,...
                data(inlierIdx,1:2),data(inlierIdx,3:4),...
                data(outlierIdx,1:2),data(outlierIdx,3:4));
            
    % return well matched points for 3D pairing
    best_match1 = data(inlierIdx,1:2);
    best_match1 = xy2idx(best_match1(:,1), best_match1(:,2));
    
    best_match2 = data(inlierIdx,3:4);
    best_match2 = xy2idx(best_match2(:,1), best_match2(:,2));
end