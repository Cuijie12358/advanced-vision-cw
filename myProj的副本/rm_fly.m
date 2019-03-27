function idx_fly = rm_fly(pc)

    % reshape the location into [480, 640, 3] matrix
    location = pc.Location;
    x = location(:,1);
    y = location(:,2);
    z = location(:,3);
    
    rec_x = reshape(x, [640, 480])';
    rec_y = reshape(y, [640, 480])';
    rec_z = reshape(z, [640, 480])';
    xyz = cat(3, rec_x, rec_y, rec_z);
    
    [FX,FY] = gradient(rec_z);
    
%     filter2(,z);
    idx_fly = [];
    idx_fly = union(find(abs(FX)'>0.02), find(abs(FY)'>0.02));
    
    
    
end