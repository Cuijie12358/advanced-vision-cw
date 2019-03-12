%% Try to remove Bob from the scene 
%  This is the same as the function file. We want to show the performance.

office = load('office1.mat');
office = office.pcl_train;
rgb = office{27}.Color;
position = office{27}.Location;

image_bob = imag2d2(rgb);
rgb_bob = im2double(image_bob);
gray_bob = rgb2gray(image_bob);
hsv_bob = rgb2hsv(rgb_bob);
lab_bob = rgb2lab(rgb_bob);


bob_trousers = 255-gray_bob;
tr_trousers = 230;
bob_trousers(bob_trousers<=tr_trousers)=0.0;
b_tr_clean = bwareaopen(bob_trousers,800);
imshow(b_tr_clean);
figure(1)

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
figure(2)
imshow(bob_bd_last)

SE2 = strel('square', 5);
SE_erode = strel('line',10,0);

edge_bob = edge(rgb_bob(:,:,1),'Canny',0.1);
bob_shirt = imdilate(edge_bob, SE2);
bob_erode = imerode(bob_shirt,SE_erode);

b_sh_clean = bwareaopen(bob_erode,8000);
bob_sh_last = imclose(b_sh_clean,SE);
err_bob = bob_sh_last+bob_bd_last+b_tr_clean;
figure(3),imshow(err_bob);

index_err_bob = find(err_bob'>0);

%
index_indoor = find(position(:,3)<3);
index_itsect = intersect(index_indoor,index_err_bob);
position(index_err_bob,:) =[]; 
% position(index_itsect,:) =[]; 

rgb(index_err_bob ,:) =[];
% rgb(index_itsect ,:) =[];
%Creating a new point -cloud
indx_xyz_no = find(position(:,3)>threshold);

position(indx_xyz_no,:) = [];
rgb(indx_xyz_no,:) = [];
new_pc = pointCloud(position, 'Color', rgb);
figure(4)
pcshow(new_pc)

