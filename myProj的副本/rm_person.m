% Parameters
%   pc:       one frame point cloud
%   manually: 0 morphology method, 1 use specific setting, 2 use mouse selection
% Return
%   idx_person: the person's indexes to remove 

function idx_person = rm_person(pc, manually)
    if (nargin<2)
        manually = 0;
    end
    
    if manually == 1
        % the range of bob is a rectangle [1:480, 91:320]
        idx = reshape(1:640*480,[640,480])';
        idx = idx(:,91:320);
        idx_person = reshape(idx,[1,230*480]);
        return
    end

    if manually == 2
        % let user select point manually
        figure;
        
        imshow(img2d(pc.Color));
        disp('please select two points')
        [x1,y1] = ginput(1);
        disp('    first point recorded')
        [x2,y2] = ginput(1);
        disp('    second point recorded')
        % limit point range
        x1 = floor(min(max(x1,1),640));
        x2 = floor(min(max(x2,1),640));
        y1 = floor(min(max(y1,1),480));
        y2 = floor(min(max(y2,1),480));
        
        close
        disp('finished')
        % choose the remove region based on selected points
        idx = 1:640*480;
        idx = reshape(idx,[640,480])';
        idx = idx(min(y1,y2):max(y1,y2), ...
                    min(x1,x2):max(x1,x2));
        
%         %show clip result
%         rgb = img2d(pc.Color);
%         rgb(y1:y2,x1:x2,:)=0;
%         imshow(rgb);
        height = abs(y1-y2)+1;
        width  = abs(x1-x2)+1;
        idx_person = reshape(idx,[1,height * width]);
        return
    end
        
    % rgb = pc.Color;
    % position = pc.Location;

    image_bob = img2d(pc.Color);
    rgb_bob = im2double(image_bob);
    gray_bob = rgb2gray(image_bob);
    lab_bob = rgb2lab(rgb_bob);

    % Remove Bobr's trousers
    bob_trousers = 255-gray_bob;
    tr_trousers = 230;
    bob_trousers(bob_trousers<=tr_trousers)=0.0;
    b_tr_clean = bwareaopen(bob_trousers,800);
    % imshow(b_tr_clean);
    % figure(1)

    % Bob's head and hands
    bob_body = lab_bob(:,:,3);
    tr_body = 4.9;
    bob_body(bob_body<tr_body)=0;
    b_bd_clean = bwareaopen(bob_body,800);
    SE = strel ('square' ,50);
    bd_imclose = imclose(b_bd_clean,SE);
    [lbl,num] = bwlabel(bd_imclose);
    lbl(lbl>2)=0;
    lbl(lbl>1)=1;
    bob_bd_last = lbl;
    % figure(2)
    % imshow(bob_bd_last)

    % Bob's shirt
    SE2 = strel('square', 5);
    SE_erode = strel('line',10,0);
    edge_bob = edge(rgb_bob(:,:,1),'Canny',0.1);
    bob_shirt = imdilate(edge_bob, SE2);
    bob_erode = imerode(bob_shirt,SE_erode);
    b_sh_clean = bwareaopen(bob_erode,8000);
    bob_sh_last = imclose(b_sh_clean,SE);
    err_bob = bob_sh_last+bob_bd_last+b_tr_clean;
    % figure(3),imshow(bob_sh_last);
    % figure(4),imshow(err_bob)
    idx_person = find(err_bob'>0);
    index_indoor = find(pc.Location(:,3)<3);
    idx_person = intersect(index_indoor,idx_person);
end