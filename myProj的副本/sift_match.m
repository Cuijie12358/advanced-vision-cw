function [best_match1, best_match2] = sift_match(pc1, pc2, filt1, filt2, frame)
    img1 = img2d(pc1.Color);
    img2 = img2d(pc2.Color);
    
    % find features and descriptors for each image
    [feat1, descr1] = vl_sift(single(rgb2gray(img1)),'EdgeThresh', 100);
    [feat2, descr2] = vl_sift(single(rgb2gray(img2)),'EdgeThresh', 100);
    
    % match these features
    [matches, scores] = vl_ubcmatch(descr1, descr2, 0.8);
    % sort the results
    [~, I] = sort(scores);
    matches = matches(:,I);
    
    matched_feat1 = feat1(:,matches(1,:));
    matched_feat2 = feat2(:,matches(2,:));
    
    % package matched features, shape M*(x1,y1,x2,y2)
    data = [matched_feat1(1:2,:)', matched_feat2(1:2,:)'];
    
%         draw_sift(img1,img2,...
%                 data(:,1:2),data(:,3:4),...
%                 data(:,1:2),data(:,3:4));
%     pause
    data = clean_match_results(data, filt1, filt2);
%     % convert xy to index in 3D
%     match1_idx = xy2idx(matched_feat1(1,:), matched_feat1(2,:));
%     match2_idx = xy2idx(matched_feat2(1,:), matched_feat2(2,:));
%     % remove points that is NaN in 3D
%     idx_rm1 = find(any(isnan(filt1.Location(match1_idx,:)), 2));
%     idx_rm2 = find(any(isnan(filt2.Location(match2_idx,:)), 2));
%     idx_rm = union(idx_rm1, idx_rm2);
%     
%     matched_feat1(:,idx_rm) = [];
%     matched_feat2(:,idx_rm) = [];
%     
%     
%     % remove repeat points
%     [~,ia1,~] = unique(matched_feat1(1,:));
%     [~,ia2,~] = unique(matched_feat2(1,:));
%     idx_repeat = intersect(ia1,ia2);
%     matched_feat1 = matched_feat1(:,idx_repeat);
%     matched_feat2 = matched_feat2(:,idx_repeat);
    
    
%     draw_sift(img1,img2,...
%             matched_feat1(1:2,1:100)',matched_feat2(1:2,1:100)',...
%             matched_feat1(1:2,:)',matched_feat2(1:2,:)');
%     drawnow;
       
    
    % RANSAC to remove outliers
    [inlierIdx, outlierIdx] = ransac_pairs(data,frame);

%     % select top 20% pairs for 3D matching
%     ratio = 1;
%     num_top = max(4, floor(ratio * length(inlierIdx)));
%     best_inlierIdx = inlierIdx(1:num_top);
%     rest_inlierIdx = inlierIdx(num_top:end);
    
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