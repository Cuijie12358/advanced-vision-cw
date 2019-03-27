function draw_sift(img1, img2, best1, best2, matched1, matched2)
    % img1, img2 two images with the same shape
    % pts1, pts2 has shape [x1 y1; x2 y2; ...] = N*2 ref point and stgt points

%     assert(size(img1) ~= size(img2));

    new_image = [img1,img2];
    new_size = size(new_image);
    half_width = new_size(2)/2;

    best2(:,1)    = best2(:,1)    + half_width;
    matched2(:,1) = matched2(:,1) + half_width;
    
    figure(3)
    imshow(new_image)
    hold on
    for i = 1: length(matched1)
       plot(matched1(i,1),matched1(i,2),'xr');
       plot(matched2(i,1),matched2(i,2),'xr');
    end
    for i = 1:length(best1)
       plot([best1(i,1),best2(i,1)], [best1(i,2),best2(i,2)],'y','LineWidth',0.6);
       plot(best1(i,1),best1(i,2),'og');
       plot(best2(i,1),best2(i,2),'og');
    end
    hold off
    
end