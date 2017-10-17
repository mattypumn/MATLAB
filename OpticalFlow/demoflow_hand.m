clear; close all; clc;

%% Import needed folders
addpath('mex');
% Using MARS Matlab.
mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));
% Using ExtractViconToCameraPoses.m 
addpath('~/redwood_ws/MATLAB/');

%% Parameters
dataset_dir = '~/for_matt/pixel_finger/color/exp5/';
crop_height = 416;   % Use full height of image if no color problems in image.
start_image = 500;
end_image = 2000;
I_q_C = [-0.9999381563640312;  0.004447440852834265;  -0.009283474950362221;  0.004209609246604872];
I_p_C =  [-0.008010455819607585;  0.00785687880804543;  -0.008900038558237013];
fc = [480.9527955850893;  480.74901328696];
cc = [319.3068719746725; 242.9280558633993];
kc = [0.01613255596460211; -0.06416463599461311; 0.1025574979617985];

rescale_factor = 0.5;
fc = fc * rescale_factor;
cc = cc * rescale_factor;


%% Setup.
image_folder = fullfile(dataset_dir, 'dump', 'feature_tracking_primary');
imagename_template = 'image_%05d.ppm';
vicon_target_file = fullfile(dataset_dir, 'axe_pose.txt');
time_alignment_file = fullfile(dataset_dir, 'time_alignment.txt');
tango_to_vicon_calib_filepath = ...
        fullfile('~/for_matt/pixel_finger/color/exp5/',...
                 'Tango_to_Vicon_Calibration.txt');
image_timestamps_file = fullfile(dataset_dir, 'dump', ...
                            'feature_tracking_primary_timestamps.txt');

%% Load transformation file.
[img_nums, v_p_c, v_q_c] = ExtractViconToCameraPoses(vicon_target_file, ...
                                  time_alignment_file, ...
                                  tango_to_vicon_calib_filepath, ...
                                  image_timestamps_file, ...
                                  I_p_C, I_q_C);
transform_data = [img_nums, v_p_c, v_q_c];

if (end_image >= img_nums(end))
    end_image = img_nums(end-1);
end
mask_fig = figure();
% mesh_fig = figure();
% flow_fig = figure();
%% Main loop.
for n = start_image : end_image
    n1 = n;
    n2 = n + 1;

    im1_file = fullfile(image_folder, sprintf(imagename_template, n1));
    im_disp = imread(im1_file);
    im_disp = im_disp(1:crop_height, :, :);
    im1 = im2double(imread(im1_file));
    im2_file = fullfile(image_folder, sprintf(imagename_template, n2));
    im2 = im2double(imread(im2_file));
    im1 = rgb2gray(im1(1:416,:,:));
    im2 = rgb2gray(im2(1:416,:,:));
    
    % When scaling, we must scale fc, cc equally.
    im1 = imresize(im1, rescale_factor);
    im2 = imresize(im2, rescale_factor);

    % set optical flow parameters (see Coarse2FineTwoFrames.m for the definition of the parameters)
    alpha = 0.012;
    ratio = 0.75;
    minWidth = 20;
    nOuterFPIterations = 7;
    nInnerFPIterations = 1;
    nSORIterations = 30;

    para = [alpha,ratio,minWidth,nOuterFPIterations,nInnerFPIterations,nSORIterations];

    [vx,vy,warpI2] = Coarse2FineTwoFrames(im1, im2, para);

    %% Get tranformation from n to n + 1
    %  Recall: transform_data = [image_nums, v_p_c, v_q_c];
    v_T_c1 = transform_data((transform_data(:,1) == n1), :);
    v_T_c2 = transform_data((transform_data(:,1) == n2), :);
    v_p_c1 = v_T_c1(2:4)';
    v_q_c1 = v_T_c1(5:end)';
    v_p_c2 = v_T_c2(2:4)';
    v_q_c2 = v_T_c2(5:end)';
    c2_p_c1 = quat2rot(v_q_c2)' * (v_p_c1 - v_p_c2);
    c2_R_c1 = quat2rot(v_q_c2)' * quat2rot(v_q_c1);

    
%     flow(:,:,1) = vx;
%     flow(:,:,2) = vy;
%     imflow = flowToColor(flow);
%     figure(flow_fig);imshow(imflow);
%     
%     vv = vx.^2 + vy.^2;
%     figure(mesh_fig), mesh(vv);
    % 
    %% Build the mask based on epiplor
    flow_mask = zeros(size(im1));
    for row = 1 : size(im1, 1)
        for col = 1 : size(im1, 2)
            pix1 = [col, row]';
            c1_ray = UndistortPoly3(pix1, fc, cc, kc);
            c1_b_1 = c1_ray / norm(c1_ray);

            pix2 = pix1 + [vx(row,col), vy(row,col)]';
            c2_ray = UndistortPoly3(pix2, fc, cc, kc);
            c2_b_2 = c2_ray / norm(c2_ray);

            t_norm = c2_p_c1 / norm(c2_p_c1); 
            flow_mask(row,col) = PassesEpipolarConstraint(c1_b_1, ...
                                                c2_b_2, c2_R_c1, t_norm);
        end
    end
    
    [Gmag,Gdir] = imgradient(im1);
    imagesc(((Gmag > .03) .* flow_mask) > 0.002)
    
%     figure(mask_fig); 
%     idx = find(flow_mask);
%     disp_r = im_disp(:, :, 1); disp_r(idx) = 255;
%     im_disp(:,:,1) = disp_r;
%     imshow(im_disp);
    
    
%     disp('Enter to continue...');
%     pause;
end






%% 

function ray = UndistortPoly3(pixel, fc, cc, kc)
    kMaxIters = 20;
    curr_perterbed = (pixel - cc) ./ fc;
    prev_perterbed = curr_perterbed;
    
    for iter = 1 : kMaxIters
        r2 = norm(curr_perterbed) ^2;
        denom = 1 + r2 * (kc(1) + r2 * (kc(2) + r2 * kc(3)));
        curr_perterbed = prev_perterbed / denom;
    end
    ray = [curr_perterbed; 1];
end

function [passes] = PassesEpipolarConstraint(c1_b_1, c2_b_2, c2_R_c1, c2_t_c1, thresh)
    kEpipolarThresh = 0.005;
    if exist('thresh', 'var')
        kEpipolarThresh = thresh;
    end
%     c1_b_1_norm = c1_b_1 / norm(c1_b_1);
%     c2_b_2_norm = c2_b_2 / norm(c2_b_2);
%     c2_t_c1_norm = c2_t_c1 / norm(c2_t_c1);
    
    err = c2_b_2' * skewsymm(c2_t_c1) * c2_R_c1 * c1_b_1;
%     passes = abs(err) < kEpipolarThresh;
    passes = abs(err);
end








