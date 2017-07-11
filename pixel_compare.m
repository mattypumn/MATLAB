close all; clear; clc;

%% Parameters.
start_image = 6;
end_image = 821;
dataset = '~/for_matt/polaris/pr55_ws/45deg/';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fast_points_dir = '/usr/local/google/home/mpoulter/Desktop/cvfast/points/';
comp_points_dir = '/usr/local/google/home/mpoulter/Desktop/fast_module/points/';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fast_points_dir = '~/for_matt/fast_points_pr55_kitchen_mrinalexp/';
% comp_points_dir = '~/for_matt/fast_points_pr55_kitchen_45_548pm/';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Setup.
% updated_dir = '~/for_matt/height_out/updated_extracted_points/';
image_dir = [dataset 'dump/feature_tracking_primary/'];
image_timestamps_file = [dataset 'dump/feature_tracking_primary_timestamps.txt'];
% feature_points_timestamps = '/usr/local/google/home/mpoulter/for_matt/height_out/feature.plane_stereo.point_label_map.txt';
% updated_points_timestamps = '/usr/local/google/home/mpoulter/for_matt/height_out/timestamps_to_count.txt';


%% Read timestamp data.
% image_timestamp_map = dlmread(image_timestamps_file);
fid = fopen(image_timestamps_file);
tline = fgetl(fid);
image_timestamp_map = [];
while( ischar(tline))
    line_arr = strsplit(tline, ' ');
    img_str = line_arr{1};
    img_num = str2double(img_str(7:11));
    
    time_str = line_arr{2};
    time_num = str2double(time_str);
    image_timestamp_map = [image_timestamp_map; img_num, time_num]; 
    
    tline = fgetl(fid);
end
fclose(fid);


%%  Main Loop over all of the images within range.
fig = figure();
for iter = start_image : end_image
   % Build the strings for the points_file and image_file.
   feature_points_file = [fast_points_dir 'points_' num2str(iter, '%06f') '.txt'];
   second_points_file = [comp_points_dir num2str(iter, '%06f') '.txt'];
   image_file = [image_dir 'image_' num2str(iter, '%05d') '.pgm'];

%    timestamp = feature_timestamp_map(i,1);
%    image_idx = find(abs(str2double(image_timestamp_map(:,2)) - timestamp) < 1e-5);
%    cell = image_timestamp_map(image_idx, 1);
%    image_file = [image_dir cell{1}];
%    
%    updated_idx = find(abs(str2double(image_timestamp_map(:,2)) - timestamp) < 1e-5);
%    updated_points_num = updated_timestamp_map(updated_idx,2);
%    updated_points_file = [updated_dir 'points_' num2str(updated_points_num) '.txt'];
%    
   im = imread(image_file);
   feature_points = dlmread(feature_points_file);
   comp_points = dlmread(second_points_file);
%    updated_points = dlmread(updated_points_file);
   
nums = [];
cell_h = 400/4;
cell_w = 640/4;
for i=1:4
    h_start = (i-1)*cell_h + 1;
    h_end = h_start + cell_h - 1;
    pts = comp_points(comp_points(:,2) <= h_end & ...
                      comp_points(:,2) >= h_start, ...
                      :);
    for j=1:4
        w_start = (j-1)*cell_w + 1;
        w_end = w_start + cell_w - 1;
        pts2= pts(pts(:,1) <= w_end & ...
                  pts(:,1) >= w_start, ...
                  :);
        nums(j,i) = size(pts2,1);
    end
end

nums
        
    





   disp(['cvfast count: ' num2str(size(feature_points,1)) '  fast-module: ' num2str(size(comp_points,1))]);
   imshow(im);
   set(fig, 'units','normalized','outerposition',[0 0 1 1]);
   hold on;
   plot(feature_points(:,1)', feature_points(:,2)', 'xr');
   plot(comp_points(:,1)', comp_points(:,2)', 'ob');
%    plot(updated_points(:,1)', updated_points(:,2)', 'ob');

   legend('mrinal-experimental-branch', 'new-module');
   axis equal;
   
   pause();
   hold off
%    close all;
end
