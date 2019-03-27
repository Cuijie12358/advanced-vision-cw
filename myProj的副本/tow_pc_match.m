function tform = tow_pc_match(pc_ref1, pc_ref2, pc_ref_new1, pc_ref_new2, frame)
    if ismember(frame,[22,24,30,32])
        % SURF matching
        [best_match1a, best_match2a] = surf_match(pc_ref1, pc_ref2,...
            pc_ref_new1, pc_ref_new2, frame);
        
        [best_match1b, best_match2b] = sift_match(pc_ref1, pc_ref2,...
            pc_ref_new1, pc_ref_new2, frame);
        
        best_match1 = [best_match1a; best_match1b];
        best_match2 = [best_match2a; best_match2b];

    else
        % SIFT matching
        [best_match1, best_match2] = sift_match(pc_ref1, pc_ref2,...
            pc_ref_new1, pc_ref_new2, frame);
    end
    
    % calculate transformation maxtrix
    fixed = select(pc_ref_new1, best_match1);
    moving = select(pc_ref_new2, best_match2);
    tform = pcregistericp(moving,fixed);
    
    % convert moving pc to origin coordinate system
    pc_aligned = pctransform(pc_ref_new2,tform);
    
    % apply point cloud merge
    mergeSize = 0.01;
    pc_merged = pcmerge(pc_ref_new1, pc_aligned, mergeSize);
    
    figure(2)
    pcshow(pc_merged);
end