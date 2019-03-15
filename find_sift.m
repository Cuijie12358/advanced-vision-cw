function [best_matches, best_PosR_index, best_PosC_index, best_Count]= find_sift(ptCloudRef,ptCloudCurrent,err_th,point_num)
%  This function is used to find the best matched pairs of two pointcloud
%  images. We use vl_ubcmatch() to find the sift pairs and use RANSAC to
%  find the best data point for these two images.
%  Detailed explanation: 
%     best_matches:  matched index of the sift points listed in vl_sift.
%     best_PosR_index: best index of pointcloud in reference frame.
%     best_PosC_index: best index of pointcloud in current frame.
%     best_Count: number of the points matched in these two images.
%     still in test .
    if (nargin<3)
        point_num = 4;

        err_th = 0.26;
    end
    if (nargin<4)
%         iter_time = 20000;
        point_num = 4;
    end
%     if (nargin<5)
%         err_th = 0.1;
%     end
    pcRLocation = ptCloudRef.Location;
    pcCLocation = ptCloudCurrent.Location;
    imgCloudRef = imag2d2(ptCloudRef.Color);
    imgCloudCurrent = imag2d2(ptCloudCurrent.Color);
    IRef = single(rgb2gray(imgCloudRef));
    ICurrent = single(rgb2gray(imgCloudCurrent));
    [fRef, dRef] = vl_sift(IRef);
    [fCurrent, dCurrent] = vl_sift(ICurrent);
    [matches, scores] = vl_ubcmatch(dRef, dCurrent);
    [~,matchesid] = sort(scores);
    matches = matches(:,matchesid);
    iter_time = 30000;
%     iter_time = 200000;

    % 
    % subplot(1,2,1),imshow(imgCloudRef)
    % hold on 
    % plot(fRef(1,:),fRef(2,:),'.','color','red')
    % hold off
    % 
    % subplot(1,2,2),imshow(imgCloudCurrent);
    % hold on 
    % plot(fCurrent(1,:),fCurrent(2,:),'.','color','red')
    % hold off


    % Top_matches = matches(:,scores< 700); %Find the most similar points between images.
    pointRef = matches(1,:);
    pointCur = matches(2,:);
%     PosR_index = diag(repmat([1,640],size(pointRef,2),1) * floor(fRef(1:2,pointRef)));
%     PosC_index = diag(repmat([1,640],size(pointCur,2),1) * floor(fCurrent(1:2,pointCur)));
    
    PosR_index = diag(repmat([1,640],size(pointRef,2),1) * (floor(fRef(1:2,pointRef))+[1;0]));
    PosC_index = diag(repmat([1,640],size(pointCur,2),1) * (floor(fCurrent(1:2,pointCur))+[1;0]));


    Ref_index = ~isnan(pcRLocation(PosR_index,1)) & ~isnan(pcRLocation(PosR_index,2)) & ~isnan(pcRLocation(PosR_index,3)); 
    Cur_index = ~isnan(pcCLocation(PosC_index,1)) & ~isnan(pcCLocation(PosC_index,2)) & ~isnan(pcCLocation(PosC_index,3));
    sel_index = Ref_index & Cur_index;
    matches = matches(:,sel_index~=0);
    pointRef = matches(1,:);
    pointCur = matches(2,:);
    xy_Ref = floor(fRef(1:2,pointRef));
    xy_Cur = floor(fCurrent(1:2,pointCur));
    PosR_index = diag(repmat([1,640],size(pointRef,2),1) * (floor(fRef(1:2,pointRef))+[1;0]));
    PosC_index = diag(repmat([1,640],size(pointCur,2),1) * (floor(fCurrent(1:2,pointCur))+[1;0]));

%     PosR_index = diag(repmat([1,640],size(pointRef,2),1) * (floor(fRef(1:2,pointRef))-[0;1]));
%     PosC_index = diag(repmat([1,640],size(pointCur,2),1) * (floor(fCurrent(1:2,pointCur))-[0;1]));

    index_allR4 = horzcat(pcRLocation(PosR_index,:),ones(length(PosR_index),1)); %Nx4  
    index_allC4 = horzcat(pcCLocation(PosC_index,:),ones(length(PosC_index),1));


    PosR_index1 = [];
    PosC_index1 = [];
    best_matches1 = [];
    best_Count = 999;
    best_matches = [];
    for times = 1:2
        i = 0;
%         point_num = 5 * times + 5 + floor(rand(1)*6);
%         point_num = 10 + floor(rand(1)*11);
        best_Count1 = 999;
        
        while i < iter_time
            temp = 0;
            W = zeros(4);
            perm = randperm(min(150,length(matches)));
            sel = perm(1:point_num);
%             pointRef = matches(1,sel);
%             pointCur = matches(2,sel);
%             Ref_index = ~isnan(pcRLocation(pointRef,3)); 
%             Cur_index = ~isnan(pcCLocation(pointCur,3));
%             sel_index = Ref_index & Cur_index;
%             sel = sel(sel_index~=0);
            pointRef = matches(1,sel);
            pointCur = matches(2,sel);
%             PosR_index = diag(repmat([1,640],size(pointRef,2),1) * floor(fRef(1:2,pointRef)));
%             PosC_index = diag(repmat([1,640],size(pointCur,2),1) * floor(fCurrent(1:2,pointCur)));

            PosR_index = diag(repmat([1,640],size(pointRef,2),1) * (floor(fRef(1:2,pointRef))+[1;0]));
            PosC_index = diag(repmat([1,640],size(pointCur,2),1) * (floor(fCurrent(1:2,pointCur))+[1;0]));
%  
% 
%             
%             
            
%             if (times==2)
%                 while(min(fRef(2,pointRef)-fCurrent(2,pointCur))<0)
%                     if (temp>10)
%                         break;
%                     end
%                     reserve_pR = pointRef(fRef(2,pointRef)-fCurrent(2,pointCur)>0);
%                     reserve_pC = pointCur(fRef(2,pointRef)-fCurrent(2,pointCur)>0);
%                     perm = randperm(min(150,length(matches)));
%                     sel = perm(1:point_num-length(reserve_pR));
%                     pointRef = [reserve_pR,matches(1,sel)];
%                     pointCur = [reserve_pC,matches(2,sel)];
%                     PosR_index = diag(repmat([1,640],size(pointRef,2),1) * (floor(fRef(1:2,pointRef))+[1;0]));
%                     PosC_index = diag(repmat([1,640],size(pointCur,2),1) * (floor(fCurrent(1:2,pointCur))+[1;0]));
%                     temp = temp +1;                    
% 
%                 end
%             end            
%             if (times==1 )
%                 while(max(fRef(2,pointRef)-fCurrent(2,pointCur))>0)
%                     if (temp>10)
%                         break;
%                     end                    
%                     reserve_pR = pointRef(fRef(2,pointRef)-fCurrent(2,pointCur)<0);
%                     reserve_pC = pointCur(fRef(2,pointRef)-fCurrent(2,pointCur)<0);
%                     perm = randperm(min(150,length(matches)));
%                     sel = perm(1:point_num-length(reserve_pR));
%                     pointRef = [reserve_pR,matches(1,sel)];
%                     pointCur = [reserve_pC,matches(2,sel)];
%                     PosR_index = diag(repmat([1,640],size(pointRef,2),1) * (floor(fRef(1:2,pointRef))+[1;0]));
%                     PosC_index = diag(repmat([1,640],size(pointCur,2),1) * (floor(fCurrent(1:2,pointCur))+[1;0]));
%                     temp = temp +1;
% 
%                 end
%                 
%             end

            
            
            
            %SVD
            index_R4 = horzcat(pcRLocation(PosR_index,:),ones(length(PosR_index),1)); %Nx4  
            index_C4 = horzcat(pcCLocation(PosC_index,:),ones(length(PosC_index),1));
            mu_pR = mean(index_R4,1);
            mu_pC = mean(index_C4,1);
            Norm_R4 = index_R4-repmat(mu_pR,size(index_R4,1),1);
            Norm_C4 = index_C4-repmat(mu_pC,size(index_R4,1),1);
            W =  Norm_R4'* Norm_C4;
            if(max(isnan(W)))
                pause
            end
   

            [U,S,V] = svd(W);
            R_ =  U * V';
            T_ = mu_pC' - R_ * mu_pR';
            Test_Cur_all = R_* index_allR4' + repmat(T_,1,size(index_allR4,1)); %4xN
            Error = sum((Test_Cur_all' - index_allC4).^2,2);
%             Test_Ref_all = R_* index_allC4' + repmat(T_,1,size(index_allC4,1)); %4xN
%             Error = sum((Test_Ref_all' - index_allR4).^2,2);

            Error = sqrt(Error);
%             Error = norm(Test_Cur_all' - index_allC4);
%             Count_now = size(Error(Error < err_th),1);
            Eval_count = Error/err_th;
            Eval_count(Eval_count>1)=1;
            Count_now = sum(Eval_count);
            
            if(best_Count1 > Count_now)
                best_Count1 = Count_now;
                best_matches1 = matches(:,Error < err_th);
                
                pointR_matched1 = pointRef;
                pointC_matched1 = pointCur;                
                pointRef = best_matches1(1,:);
                pointCur = best_matches1(2,:);
%                 PosR_index1 = diag(repmat([1,640],size(pointRef,2),1) * floor(fRef(1:2,pointRef)));
%                 PosC_index1 = diag(repmat([1,640],size(pointCur,2),1) * floor(fCurrent(1:2,pointCur)));
                PosR_index1 = diag(repmat([1,640],size(pointRef,2),1) * (floor(fRef(1:2,pointRef))+[1;0]));
                PosC_index1 = diag(repmat([1,640],size(pointCur,2),1) * (floor(fCurrent(1:2,pointCur))+[1;0]));


            end
          

            i = i+1;

        end
        best_Count1
        point_num
        
        if (best_Count>best_Count1)
            best_PosR_index = PosR_index1;
            best_PosC_index = PosC_index1;
            best_Count = best_Count1;
            best_matches = best_matches1;
            pointRef = best_matches1(1,:);
            pointCur = best_matches1(2,:);
            pointR_matched = pointR_matched1;
            pointC_matched = pointC_matched1;
            
        end    
        
    end
    draw_sift(imgCloudRef,imgCloudCurrent,floor(fRef(1:2,pointR_matched))',floor(fCurrent(1:2,pointC_matched))',xy_Ref',xy_Cur')
%     pause
    
end

