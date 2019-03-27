function [inlierIdx, outlierIdx]= ransac_pairs(data,frame)

    if ismember(frame,[22, 24,30,32])
        maxError = 30;
        MaxNumTrials=10000;
    end

    sampleSize = 4; % number of points to sample per trial
    maxError = 10; % max allowable errors for inliers
    MaxNumTrials = 5000;
    % function to calculate 2D points transform
    tfromFunc = @(data) fitgeotrans(data(:,1:2),data(:,3:4),'affine');
    % function to calculate the loss after one trail
    function out = evaluate(model,data)
        points1 = [data(:,1:2),ones(length(data),1)];
        points2 = [data(:,3:4),ones(length(data),1)];
        distance = points1*model.T-points2;
        out = sum(distance(:,1:2).^2,2);
        % focus penalty
%         good = data(sum(distance(:,1:2).^2)<=5,:);
%         out = out + 640/max(abs(good(:,1)-good(:,3)))+ ...
%                     480/max(abs(good(:,2)-good(:,4)));
    end
    evalFunc = @evaluate;
    
    try
        [~,inlierIdx] = ransac(data,tfromFunc,evalFunc,sampleSize,maxError,'MaxNumTrials',MaxNumTrials);
    catch
        maxError = 50;
        [~,inlierIdx] = ransac(data,tfromFunc,evalFunc,sampleSize,maxError,'MaxNumTrials',100000);
    end
    %,'MaxSamplingAttempts',1000, 'MaxNumTrials',100000);
    outlierIdx = setdiff(1:length(data),find(inlierIdx));
end