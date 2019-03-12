%% Extract the relevant data from each point cloud
%  This part is the first part of the task, we preprocesing the point cloud
%  data.
%
%  indx_err_bob(list) : detecting and find Bob in the 27th frame.

%  indx_xyz_no(list) : we threshold the distance of the
%  data within 3.5 meters.

%  indx_xyz_edge(list) : We remove the points at the edge of the 2d
%  images.image2d(480,:);image2d(:,1);image2d(:,640);image2d(1,:)

%  outlierIndices(list) : The flying points which have their average
%  distance between 4 neighbours greater than 0.8
%
%  we fill the x,z,y with NaN in these point.

office = load('office1.mat');
office = office.pcl_train;
rgb = [];
point = [];
indx_bob = [];
indx_xyz_edge = [];

threshold = 3.5;
office_filtered = {};
for i = 1:length(office) % Reading the 40 point-clouds
%     pcshow(office{i});
%     imgrgb = imag2d2(office{i}.Color);
%     imshow(imgrgb)
%   
    if (i == 27)
        indx_err_bob = rm_bob(office{i});
    else
        indx_err_bob = [];
    end

    rgb =  office{i}.Color; % Extracting the colour data
    point =  office{i}.Location; % Extracting the xyz data
    indx_xyz_no = find(point(:,3)>threshold);%(Code: find the indices of the indoor pixels.) 
    
    
    image2d = rgb2gray(imag2d2(rgb));
    indx_xyz_edge = [find(image2d(1,:))' ;find(image2d(480,:))' ;find(image2d(:,1));find(image2d(:,640))];

    index_rm = union(indx_xyz_no,indx_xyz_edge);
    index_rm = union(index_rm,indx_err_bob);

    new_pc = pointCloud(point, 'Color', rgb);
    [~,inlierIndices,outlierIndices] = pcdenoise(new_pc,'NumNeighbors',4,'Threshold',0.8); % Flying points
    index_rm = union(index_rm,outlierIndices);
    point(index_rm,:)= NaN;
    rgb(index_rm,:)= 0; 
    new_pc = pointCloud(point, 'Color', rgb);
    office_filtered{end+1} = new_pc;
    
%     subplot(1,2,1),pcshow(office{i})
%     subplot(1,2,2),pcshow(new_pc)
%     pause
    
end
save('office_filtered.mat','office_filtered')
%% Estimating the Reference Frame Transformation Between Consecutive Frames
%  We use vl_sift to find the sift pairs and sort them by their scores, we
%  fins sift and use RANSAC to get the best data pairs in find_sift()

% pre-saved model
% office_filtered = load('office_filtered.mat');
% office_filtered = office.office_filtered;

ptCloudRef = office_filtered{1};
ptCloudCurrent = office_filtered{2};
[best_matches,PosR_index,PosC_index,best_Count] = find_sift(ptCloudRef,ptCloudCurrent);

% pointRef = best_matches(1,:);
% pointCur = best_matches(2,:);
% PosR_index = diag(repmat([1,640],size(pointRef,2),1) * floor(fRef(1:2,pointRef)));
% PosC_index = diag(repmat([1,640],size(pointCur,2),1) * floor(fCurrent(1:2,pointCur)));
%     
fixed_Features = pointCloud(ptCloudRef.Location(PosR_index,:), 'Color', ptCloudRef.Color(PosR_index,:));
moving_Features = pointCloud(ptCloudCurrent.Location(PosC_index,:), 'Color', ptCloudCurrent.Color(PosC_index,:));

[tform,movingReg] = pcregistericp(moving_Features, fixed_Features, 'Metric','pointToPlane','Extrapolate', true);
ptCloudAligned = pctransform(ptCloudCurrent,tform);


mergeSize = 0.01;
ptCloudScene1 = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);


figure(1)
pcshow(ptCloudScene1)
title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

%% 

ptCloudScene = ptCloudScene1;
accumTform = tform; 

figure(1)

for i = 2:length(office_filtered)-1%length(office_within3)
    
    ptCloudRef = office_filtered{i};
    ptCloudCurrent = office_filtered{i+1};
%     err_th = 0.21;
%     point_num = 10;
%     if(i>14)
%         err_th = 0.20;
%         point_num = 4;
%     end
%     if (i>16)
%         err_th = 0.9;
%         point_num = 4;
%     end
%     if (i>18)
%         err_th = 1;
%         point_num = 10;
%     end
    [best_matches,PosR_index,PosC_index,best_Count] = find_sift(ptCloudRef,ptCloudCurrent,err_th,point_num);

    fixed_Features = pointCloud(ptCloudRef.Location(PosR_index,:), 'Color', ptCloudRef.Color(PosR_index,:));
    moving_Features = pointCloud(ptCloudCurrent.Location(PosC_index,:), 'Color', ptCloudCurrent.Color(PosC_index,:));

    [tform,movingReg] = pcregistericp(moving_Features, fixed_Features, 'Metric','pointToPlane','Extrapolate', true);
    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    ptCloudAligned_now = pctransform(ptCloudCurrent, tform);
    
    % Update the world scene.
    ptCloudScene_now = pcmerge(ptCloudRef, ptCloudAligned_now, mergeSize);    
%     subplot(1,2,1),
    pcshow(ptCloudScene_now)
    
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
    
    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);

    figure(2)
    pcshow(ptCloudScene)
    title('Initial world scene')
    xlabel('X (m)')
    ylabel('Y (m)')
    zlabel('Z (m)')
    i
    if (i>10)
        pause
    end
end
%% Evaluate the quality of the final model
%  This part hasn't done yet.
% During the recording, the Kinect was pointing downward. To visualize the
% result more easily, let's transform the data so that the ground plane is
% parallel to the X-Z plane.
% angle = +pi/10;
% A = [1,0,0,0;...
%      0, cos(angle), sin(angle), 0; ...
%      0, -sin(angle), cos(angle), 0; ...
%      0 0 0 1];
% ptCloudScene = pctransform(ptCloudScene, affine3d(A));
% pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% title('Updated world scene')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')



