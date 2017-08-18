close all; clear; clc;

%% Parameters.
start_image = 488;
end_image = 821;
dataset = '~/for_matt/polaris/pr55_ws/45deg/';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fast_points_dir = '/usr/local/google/home/mpoulter/Desktop/fast_module/points/';
below_points_dir = '/usr/local/google/home/mpoulter/Desktop/fast_module/below_points/';
comp_points_dir = '/usr/local/google/home/mpoulter/Desktop/FREAK/';
horizon_dir = '/usr/local/google/home/mpoulter/Desktop/fast_module/horizon/';
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
set(fig, 'units','normalized','outerposition',[0 0 1 1]);
for iter = start_image : end_image
    %% Build the strings for the files.
    time_idx = find(image_timestamp_map(:,1) == iter);
    time = image_timestamp_map(time_idx, 2);
    fast_points_file = [fast_points_dir num2str(time, '%06f') '.txt'];
    below_points_file = [below_points_dir num2str(time, '%06f') '.txt'];
    freak_points_file = [comp_points_dir num2str(time, '%06f') '.txt'];
    horizon_file = [horizon_dir num2str(time, '%06f') '.txt'];
    image_file = [image_dir 'image_' num2str(iter, '%05d') '.pgm'];

    %% Extract data.
    fast_points = [];
    below_points = [];
    freak_points = [];
    horizon_points = [];

    im = imread(image_file);   
    
    if  exist(freak_points_file, 'file')
        freak_points = dlmread(freak_points_file);
    else 
        disp('No FREAK points.');
        continue;
    end

    if  exist(fast_points_file, 'file')
        fast_points = dlmread(fast_points_file);
    else 
        disp('No fast points.');
    end
    
    if  exist(below_points_file, 'file')
        below_points = dlmread(below_points_file);
    else 
        disp('No fast points.');
    end

    if  exist(horizon_file, 'file')
        horizon_points = dlmread(horizon_file);
    [hor1, hor2, horizon_intersects] = edge_intersections(im, ...
                            horizon_points(1,:)', horizon_points(2,:)');
    else 
        disp('No horizon line.');
        horizon_intersects = false;
    end   

    %% Ensure nothing is empty.
    if ~horizon_intersects
        hor1 = [1;1];
        hor2 = [1;1];
    end
    if isempty(freak_points) 
        freak_points = [1, 1];
    end
    if isempty(fast_points)
        fast_points = [1,1];
    end
    if isempty(below_points)
        below_points = [1,1];
    end
                        
    %% Display.
    disp(['cvfast count: ' num2str(size(fast_points,1)) ...
         '  fast-module: ' num2str(size(freak_points,1))]);                                     
  
    imshow(im);                   
    hold on; 
    line_plot = plot([hor1(1), hor2(1)], [hor1(2), hor2(2)], 'color', 'g', 'linewidth', 1);                  
    freak_plot = plot(freak_points(:,1)', freak_points(:,2)', 'ob');
    fast_plot = plot(fast_points(:,1)', fast_points(:,2)', 'xr');
    below_plot = plot(below_points(:,1)', below_points(:,2)', '+g');

    legend('Horizon', 'Freak', 'Fast', 'Fast below horizon');

   
    pause();
    delete(line_plot);
    delete(freak_plot);
    delete(fast_plot);
    delete(below_plot);
    hold off;
end

function [pix1, pix2, horizon_intersects] = edge_intersections(image, point1, point2)
    h_of_t = @(t) point1 * (1-t) + point2 * t;
    [image_height, image_width] = size(image);
    intersections = 0;
    
    top = [0, 0, 1, 0]';
    left = [0, 0, 0, 1]';
    right = [image_width, 0, 0, 1]';
    bottom = [0, image_height, 1, 0]';
    lines = [top left right bottom];
    
    for i = 1 : 4
        start_pix = lines(1:2,i);
        end_pix = start_pix + lines(3:4, i) .* [image_width; image_height];
        A = [(end_pix - start_pix), -(point2 - point1)];
        b = point1 - lines(1:2, i);
        t = A \ b;   
        
        pix_tmp = h_of_t(t(2));
        
        if pix_tmp(1) >= start_pix(1)-1 && pix_tmp(1) <= end_pix(1) + 1 ...
           && pix_tmp(2) >= start_pix(2)-1 && pix_tmp(2) <= end_pix(2) + 1
            % We have an intersection.
            if intersections == 0 
                pix1 = pix_tmp + 1;
            elseif intersections == 1 
                pix2 = pix_tmp + 1; % fix the 1 indexing in matlab.
            end    
            intersections = intersections + 1;
        end
    end

    if intersections == 2
        horizon_intersects = true;
    else 
        horizon_intersects = false;
        pix1 = [1;1];
        pix2 = [1;1];
    end
end