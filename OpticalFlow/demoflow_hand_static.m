clear; close all; clc;

%% Import needed folders
addpath('mex');
% Using MARS Matlab.
mars_matlab_path = getenv('MARS_MATLAB');
addpath(fullfile(mars_matlab_path, 'robotics3D'));
% Using ExtractViconToCameraPoses.m 
addpath('~/redwood_ws/MATLAB/');

%% Parameters
dataset_dir = '~/for_matt/pixel_finger/static_camera/exp7/';
bg_image_num = 25;
start_image = 200;
end_image = 2000;

rescale_factor = 0.5;
kMagThresh = 100;


%% Setup.
image_folder = fullfile(dataset_dir, 'dump', 'feature_tracking_cropped');
imagename_template = 'image_%05d.ppm';


%% Load background image.

bg_file = fullfile(image_folder, sprintf(imagename_template, bg_image_num));
bg_disp = imread(bg_file);
bg_im = im2double(imread(bg_file));
bg_im = rgb2gray(bg_im);
bg_fig = figure();
imshow(bg_disp);


%% Build extra figures.
mask_fig = figure();
im_fig = figure();
flow_fig = figure();
%% Main loop.
for n = start_image : end_image
    im_file = fullfile(image_folder, sprintf(imagename_template, n));
    im = im2double(imread(im_file));
    im = rgb2gray(im);
    figure(im_fig);
    imshow(im);
    
    % When scaling, we must scale fc, cc equally.
    bg_im = imresize(bg_im, rescale_factor);
    im = imresize(im, rescale_factor);

    % set optical flow parameters (see Coarse2FineTwoFrames.m for the definition of the parameters)
    alpha = 0.012;
    ratio = 0.75;
    minWidth = 20;
    nOuterFPIterations = 7;
    nInnerFPIterations = 1;
    nSORIterations = 30;

    para = [alpha,ratio,minWidth,nOuterFPIterations,nInnerFPIterations,nSORIterations];

    [vx,vy,warpI2] = Coarse2FineTwoFrames(bg_im, im, para);
    
%     flow(:,:,1) = vx;
%     flow(:,:,2) = vy;
%     imflow = flowToColor(flow);
%     figure(flow_fig);imshow(imflow);
%     
    vv = vx.^2 + vy.^2;
    figure();
    imagesc(vv);
    figure(mask_fig);
    imshow(vv > 300);
    
    
%     figure(mesh_fig), mesh(vv);
    % 
   
%     [Gmag,Gdir] = imgradient(bg_im);
%     imagesc(((Gmag > .03) .* flow_mask) > 0.002)
    
%     figure(mask_fig); 
%     idx = find(flow_mask);
%     disp_r = im_disp(:, :, 1); disp_r(idx) = 255;
%     im_disp(:,:,1) = disp_r;
%     imshow(im_disp);
    
    
%     disp('Enter to continue...');
%     pause;
end











