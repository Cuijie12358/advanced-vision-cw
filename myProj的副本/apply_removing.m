function new_pc = apply_removing(pc, idx_remove)
    location = pc.Location;
    color    = pc.Color;
    
    location(idx_remove,:) = NaN;
    color(idx_remove,:)    = NaN;
    
    new_pc = pointCloud(location, 'Color', color);
end