function draw_sift(img1, img2, pts1, pts2, pts1b, pts2b)
    % img1, img2 two images with the same shape
    % pts1, pts2 has shape [x1 y1; x2 y2; ...] = N*2 ref point and stgt points

%     assert(size(img1) ~= size(img2));

    new_image = [img1,img2];
    new_size = size(new_image);
    half_width = new_size(2)/2;

    pts2(:,1) = pts2(:,1) + half_width;
    pts2b(:,1) = pts2b(:,1) + half_width;
    figure(3)
    imshow(new_image)
    hold on
    for i = 1: length(pts1b)
       plot(pts1b(i,1),pts1b(i,2),'xr');
       plot(pts2b(i,1),pts2b(i,2),'xr');
    end
    for i = 1:length(pts1)
       plot([pts1(i,1),pts2(i,1)], [pts1(i,2),pts2(i,2)],'y','LineWidth',0.8);
       plot(pts1(i,1),pts1(i,2),'.g');
       plot(pts2(i,1),pts2(i,2),'.g');
    end
    hold off

%        plot([pts1(i,2),pts2(i,2)], [pts1(i,1),pts2(i,1)],'y','LineWidth',0.8);
%        plot(pts1(i,2),pts1(i,1),'og');
%        plot(pts2(i,2),pts2(i,1),'ob');
%        plot(pts1b(i,2),pts1b(i,1),'xr');
%        plot(pts2b(i,2),pts2b(i,1),'xr');
%     end
    
end