% return index of four edges of one image

function idx_edge = rm_edge(width, height, pixel)
%     width  = 640;
%     height = 480;
%     assert(width*height == length(pc.Location))
    idx = reshape(1:width*height, [width, height])';
    idx(pixel+1:height-pixel,pixel+1:width-pixel) = 0; 
    idx_edge = idx(idx ~= 0);
end