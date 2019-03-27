function data = clean_match_results(data, filt1, filt2)
    % convert xy to index in 3D
    match1_idx = xy2idx(data(:,1), data(:,2));
    match2_idx = xy2idx(data(:,3), data(:,4));
    % remove points that is NaN in 3D
    idx_rm1 = find(any(isnan(filt1.Location(match1_idx,:)), 2));
    idx_rm2 = find(any(isnan(filt2.Location(match2_idx,:)), 2));
    idx_rm = union(idx_rm1, idx_rm2);
    
    data(idx_rm,:) = [];
    
    % remove repeat points
    [~,ia1,~] = unique(data(:,1));
    [~,ia2,~] = unique(data(:,2));
    [~,ia3,~] = unique(data(:,3));
    [~,ia4,~] = unique(data(:,4));
    idx_repeat = intersect(ia1,ia2);
    idx_repeat = intersect(idx_repeat,ia3);
    idx_repeat = intersect(idx_repeat,ia4);
    data = data(idx_repeat,:);
end