% This is the main function for this coursework

%% Firstly, we need to clean the point cloud

% load the point cloud
office = load('office1.mat');
office = office.pcl_train;


office_filtered = {};
for i = 27:length(office)
    tmp_pc = office{i};

    idx_edge = rm_edge(640,480,5);      % remove image edge 
    idx_remote = rm_remote(tmp_pc, 3.8);   % remove remote points

    if i==27
       idx_person = rm_person(tmp_pc); % remove person in frame 27
    else
       idx_person = [];
    end

    % update the point cloud
    tmp_pc = apply_removing(tmp_pc, ...
                        union(union(idx_edge, idx_remote), idx_person));

    % remove flying points
    [~,~,idx_fly] = pcdenoise(tmp_pc,'NumNeighbors',30,'Threshold',0.8);


    % update the point cloud again
    tmp_pc = apply_removing(tmp_pc, idx_fly);

    % SHOW original images, point cloud
    figure(1);
    subplot(2,2,1);imshow(img2d(office{i}.Color));
    title('original image');
    subplot(2,2,2);pcshow(office{i});
    title('original point cloud');
    % SHOW removed points
    binaryImg = ones(640,480);
    [x_rm,y_rm] = idx2xy(union(union(union(...
                            idx_edge, idx_remote), idx_person),idx_fly));
    for k=1:length(x_rm)
       binaryImg(x_rm(k),y_rm(k))=0;
    end
    subplot(2,2,3);imshow(binaryImg');
    title('points kept(white) & ignored(black)');

    % SHOW cleaned point cloud
    subplot(2,2,4);pcshow(tmp_pc);
    title('cleaned point cloud');

    % store the filtered point cloud
    office_filtered{end+1} = tmp_pc;
    disp([int2str(i),' frame finished']);
    pause
end

%% show the result of filtered office
    
for i = 1:length(office_filtered)
    subplot(2,2,1); imshow(img2d(office_filtered{i}.Color));
    subplot(2,2,2); pcshow(office_filtered{i});
    subplot(2,2,3); imshow(img2d(office{i}.Color));
    subplot(2,2,4); pcshow(office{i});
    pause
end

%% SIFT matching test

for i=24:length(office)-1
    [best_match1, best_match2] = sift_match(office{i},office{i+1},office_filtered{i},office_filtered{i+1});

    fixed = select(office{1}, best_match1);
    moving = select(office{2},best_match2);
    tform = pcregistericp(moving,fixed);
    disp([int2str(i), ' and ', int2str(i+1), ' matching']);
%     tform
    pause
end


%% Matching and merge
accumTform = affine3d(eye(4));
pcAll = office{1};
mergeSize = 0.02;
for i=1:length(office)-1
    if i==27
        continue
    end
    if i==26 % skip frame 27
        disp([int2str(i), ' and ', int2str(i+2), ' matching']);
        tow_pc_match(office{i}, office{i+2},...
                        office_filtered{i}, office_filtered{i+2}, i);
        accumTform = affine3d(tform.T * accumTform.T);
        pcAligned = pctransform(office_filtered{i+2}, accumTform);
          
    else
    
        disp([int2str(i), ' and ', int2str(i+1), ' matching']);
        tform = tow_pc_match(office{i},office{i+1},...
                        office_filtered{i}, office_filtered{i+1}, i);
                    
        accumTform = affine3d(tform.T * accumTform.T);
        pcAligned = pctransform(office_filtered{i+1}, accumTform);
    end

    % Update the world scene.
    pcAll = pcmerge(pcAll, pcAligned, mergeSize); 
    
    figure(4);
    pcshow(pcAll);
    title('World scene')
    xlabel('X (m)')
    ylabel('Y (m)')
    zlabel('Z (m)')

    pause(0.1)
end


%% fit plane
pcAll = pcread('pcAll.ply');

% rotate model to a suitable position
anglex = pi*1.5;
angley = pi*0.95;
anglez = pi*1.2;

RX = [1,0,0,0;...
     0, cos(anglex), -sin(anglex), 0; ...
     0, sin(anglex), cos(anglex), 0; ...
     0 0 0 1];

RY = [cos(angley),0,sin(angley),0;...
     0, 1, 0, 0; ...
     -sin(angley), 0, cos(angley), 0; ...
     0 0 0 1];

RZ = [cos(anglez),-sin(anglez),0,0;...
     sin(anglez), cos(anglez), 0,  0; ...
     0, 0, 1, 0; ...
     0 0 0 1];
 
pcNew = pctransform(pcAll, affine3d(RX*RY*RZ));
subplot(2,3,1);pcshow(pcNew);
xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');
title('Original Point Cloud')


% get the right wall
maxDistance = 0.15;
referenceVector = [0,-1,0];
maxAngularDistance = 5;

[modelRight,inlierIndices1,outlierIndices1] = pcfitplane(pcNew,...
                        maxDistance,referenceVector,maxAngularDistance);
pcRight = select(pcNew,inlierIndices1);
remainPtCloud = select(pcNew,outlierIndices1);

subplot(2,3,2);pcshow(pcRight);
xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');
title('Right Wall');

% get the front wall
referenceVector2 = [1,0,0];
[modelFront,inlierIndices2,outlierIndices2] = pcfitplane(remainPtCloud,...
                        maxDistance,referenceVector2,maxAngularDistance);
pcRight = select(remainPtCloud,inlierIndices2);
remainPtCloud = select(remainPtCloud,outlierIndices2);

subplot(2,3,3);pcshow(pcRight);
xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');
title('Front Wall')


% get the left wall
referenceVector3 = [0,1,0];
[modelLeft,inlierIndices3,outlierIndices3] = pcfitplane(remainPtCloud,...
                        maxDistance,referenceVector3,maxAngularDistance);
pcLeft = select(remainPtCloud,inlierIndices3);
remainPtCloud = select(remainPtCloud,outlierIndices3);

subplot(2,3,5);pcshow(pcLeft);
xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');
title('Left Wall')


% get the ceil
referenceVector4 = [0,0,1];
[modelCeil,inlierIndices4,outlierIndices4] = pcfitplane(remainPtCloud,...
                        maxDistance,referenceVector4,maxAngularDistance);
pcCeil = select(remainPtCloud,inlierIndices4);
remainPtCloud = select(remainPtCloud,outlierIndices4);

subplot(2,3,6);pcshow(pcCeil);
xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');
title('Ceil')

% show remain point cloud
subplot(2,3,4);pcshow(remainPtCloud);
xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');
title('Remain point cloud')



%%

% calculate the product of normal vectors for each other
nRight = modelRight.Normal;
nLeft = modelLeft.Normal;
nFront = modelFront.Normal;
nCeil = modelCeil.Normal;

nArray = {nRight,nLeft,nFront,nCeil};

score = zeros(length(nArray));
for i=1:length(nArray)
    for j=1:length(nArray)
        score(i,j) = nArray{i}*nArray{j}';
    end
end

% calculate the angles between each other
score(1:length(nArray)+1:end)=1;
angles = acosd(score);










