info = load('data.mat');

i = 1;
pre = 0;
flag = 0;
pc = pointCloud(info.Point{i}, 'Color', info.RGB{i});
% pc = pcread('pc1_24.ply');
while(i < 40)
    i = i + 1;
    if flag == 0
        pre = i - 1;
    end
    if i == 27  % jump out (27,28), instead (26,28)
        i = 28;
    end
    
    flag = 0;
    [pre, i]
    % find match
    matches = vl_ubcmatch(info.D{pre}, info.D{i});
    
%     % draw match sift image
%     figure(1)
%     subplot(1,2,1)
%     img = imag2d(info.info.rgb{pre});
%     image(img);
%     h1   = vl_plotframe(info.info.frames{pre}(:,matches(1,:))) ; set(h1,'color','green','linewidth',1) ;
%     k = setdiff(info.info.frames{pre}', info.info.frames{pre}(:,matches(1,:))', 'row');
%     h2   = vl_plotframe(k') ; set(h2,'color','red','linewidth',1) ;
%     box off;
%     axis off;
%     
%     subplot(1,2,2)
%     img = imag2d(info.info.rgb{i});
%     image(img);
%     h3   = vl_plotframe(info.info.frames{i}(:,matches(2,:))) ; set(h3,'color','green','linewidth',1) ;
%     k = setdiff(info.info.frames{i}', info.info.frames{i}(:,matches(2,:))', 'row');
%     h4   = vl_plotframe(k') ; set(h4,'color','red','linewidth',1) ;
%     box off;
%     axis off;
%     filename = strcat(num2str(pre),"_",num2str(i), '.jpg');    
%     saveas(1, filename);
    
    
    % RANSAC for rotation and transform
    %calulate the mean min distance of 3d cloud pre
%     [idx, dist] = knnsearch(info.info.point{pre}, info.info.point{pre},'K',2);
%     dist = rmmissing(dist,1); % rm NaN
%     dist_threhold = man(dist(:,2));
    dist_threhold = 0.1;
    R_Finnal = [];
    T_Final = [];
    w_min = size(matches,2);
    pointp_Final = [];
    pointq_Final = [];
    if i == 25 || i == 37
        iter = 50000; % RANSAC iteration times
%     elseif i < 25
%         iter = 20000;
    else
        iter = 10000;
    end
    for k = 1:iter
        sizes = 25; 
        % caculate rotation
        index = randperm(size(matches,2), sizes); % random select 30 match points
        %find randpoint in image1
        location1 = floor(info.F{pre}(1:2, matches(1, index))); % the axis of selected points in frist image
        ps = [];
        for j = 1:sizes
            temp = (location1(2,j)-1)*640 + location1(1,j);  % index of selected point j
            ps = cat(1, ps, info.Point{pre}(temp, :));
        end
        %find randpoint in image2
        location2 = floor(info.F{i}(1:2, matches(2, index))); % the axis of selected points in frist image
        qs = [];
        for j = 1:sizes
            temp = (location2(2,j)-1)*640 + location2(1,j);  % index of selected point j
            qs = cat(1, qs, info.Point{i}(temp, :));
        end
    
        % delete nan rows
        temp = cat(1, find(isnan(ps(:,1))), find(isnan(qs(:,1))), find(isnan(ps(:,2))), find(isnan(qs(:,2))), find(isnan(ps(:,3))), find(isnan(qs(:,3))));
        ps(temp, :) = [];
        qs(temp, :) = [];
        if size(ps, 1) < 3
            continue;
        end
        
        %caculate transform R and T
        p0 = mean(ps,1);
        q0 = mean(qs,1);
        H = (ps-p0)' * (qs-q0);
        [U, S, V] = svd(H);
        R = V * U';
        T = q0' - R * p0';
        
        %calculate w_min , R_final and T_final in this iter
        w = 0;
        pointp = [];
        pointq = [];
        %find all sift point in image1
        location1 = floor(info.F{pre}(1:2, matches(1, :))); % the axis of selected points in frist image
        p = [];
        for j = 1:size(matches,2)
            temp = (location1(2,j)-1)*640 + location1(1,j);  % index of selected point j
            p = cat(1, p, info.Point{pre}(temp, :));
        end
        %find all sift point in image2
        location2 = floor(info.F{i}(1:2, matches(2, :))); % the axis of selected points in frist image
        q = [];
        for j = 1:size(matches,2)
            temp = (location2(2,j)-1)*640 + location2(1,j);  % index of selected point j
            q = cat(1, q, info.Point{i}(temp, :));
        end
        % delete nan rows
        temp = cat(1, find(isnan(p(:,1))), find(isnan(q(:,1))));
        p(temp, :) = [];
        q(temp, :) = [];
        
        for ii = 1:size(p,1)
            p_new = R * p(ii, :)' + T;
            temp = q(ii,:) - p_new';
            eudist = norm(temp);
            if eudist >= dist_threhold
                w = w + 1;
            else
                w = w + eudist / dist_threhold;
                pointp = cat(1,pointp, p(ii, :));
                pointq = cat(1,pointq, q(ii, :));
            end
        end
        if w <= w_min
            w_min = w;
            R_Finnal = R;
            T_Final = T;
            pointp_Final = pointp;
            pointq_Final = pointq;
        end
    end

    if size(pointq_Final,1) < 3 
    flag = 1;
    continue;
    %pointp_Final = p;
    %pointq_Final = q;
    end
    
    %icp
    tform = pcregistericp(pointCloud(pointp_Final), pointCloud(pointq_Final));
%   pc = pointCloud(info.info.point{i}, 'Color', info.info.rgb{i});
    pc1 = pctransform(pc, tform);
    pc2 = pointCloud(info.Point{i}, 'Color', info.RGB{i});
    pc1 = pcmerge(pc1, pc2, 0.003);
    figure(2);
    pcshow(pc1);
    if i == 25
        str = input('is this right?(y/n)','s');
        if str ~= 'y'
            i = i - 1;
            continue;
        end
    end
    pc = pc1;
%     pause
end
% pcwrite(pc, 'pc1_40(without27)');