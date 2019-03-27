% Parameters
%   pc:     one frame point cloud
%   range:  the points out of this range will be recorded
% Return
%   idx_remote: the index of remote points that to be removed

function idx_remote = rm_remote(pc, range)
    % limit the z-axis distance
    idx_remote = find(pc.Location(:,3)>range);
end