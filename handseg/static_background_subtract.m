clear; clc; close all;

%% Parameters.
dataset = '~/for_matt/pixel_finger/static_camera/exp2';
images_dir = fullfile(dataset, 'dump', 'feature_tracking_cropped');
template = 'image_%05d.ppm';
output_dir = fullfile(dataset,'dump', 'cropped_hand');
mask_template = 'mask_%05d.png';
im_out_template = 'image_%05d.png';


debug_output_dir = fullfile(dataset, 'dump', 'masked_hand');
debug_template = fullfile(debug_output_dir, 'masked_%05d.ppm');

DEBUG_IMAGES = true;
SAVE_DEBUG = false;

bg_image_num = 40;
first_image = 50;
last_image = 3000;

output_height = 240;
output_width = 240;

%% Segmenting Parameters.
% kNumSuperPixels = 500;
% kSuperPixelMinArea = 0.75;

kRgbThreshold = 10;
kHueThresh = 0.1;
kSatThresh = .01;
kUvThresh = 7;

%% Setup.
read_template = fullfile(images_dir, template);
write_template = fullfile(output_dir, im_out_template);
mask_template = fullfile(output_dir, mask_template);

system(['mkdir -p ' output_dir]);
if DEBUG_IMAGES && SAVE_DEBUG 
   system(['mkdir -p ' debug_output_dir]);
end

%% Extract Background data.
bg_rgb = imread(sprintf(read_template, bg_image_num));
bg_im_show = bg_rgb;
% bg_im = rgb2hsv(bg_im);
% bg_im(:,:,1) = medfilt2(bg_im(:,:,1));
% bg_im(:,:,2) = medfilt2(bg_im(:,:,2));
% bg_im(:,:,3) = medfilt2(bg_im(:,:,3));

bg_ycbcr = rgb2ycbcr(double(bg_rgb));
bg_ycbcr(:,:,1) = medfilt2(bg_ycbcr(:,:,1));
bg_ycbcr(:,:,2) = medfilt2(bg_ycbcr(:,:,2));
bg_ycbcr(:,:,3) = medfilt2(bg_ycbcr(:,:,3));

%% For debugging.
if DEBUG_IMAGES
    mask_fig = figure();
    sp_fig = figure();

    bg_fig = figure();
    imshow(bg_im_show);
end

%%  Main loop.
for i = first_image : last_image
    read_file = sprintf(read_template, i);
    hand_file = sprintf(write_template, i);
    mask_file = sprintf(mask_template, i);
    im_rgb = imread(read_file);
    
    r = medfilt2(im_rgb(:,:,1));
    g = medfilt2(im_rgb(:,:,2));
    b = medfilt2(im_rgb(:,:,3));
      
    im_show = im_rgb;
    im_hsv = rgb2hsv(double(im_rgb));
    im_hsv(:,:,1) = medfilt2(im_hsv(:,:,1));
    im_hsv(:,:,2) = medfilt2(im_hsv(:,:,2));
    im_hsv(:,:,3) = medfilt2(im_hsv(:,:,3));
    h = im_hsv(:,:,1);
    s = im_hsv(:,:,2);
    v = im_hsv(:,:,3);
    
    im_ycbcr = rgb2ycbcr(double(im_rgb));
    im_ycbcr(:,:,1) = medfilt2(im_ycbcr(:,:,1));
    im_ycbcr(:,:,2) = medfilt2(im_ycbcr(:,:,2));
    im_ycbcr(:,:,3) = medfilt2(im_ycbcr(:,:,3));
    
    % Calculate any background changes in the image.
    lighting_gain = ransac_background_diff(bg_ycbcr, im_ycbcr);
    
    diff = im_ycbcr - bg_ycbcr - lighting_gain;
%     diff_1norm = diff(:,:,1) + diff(:,:,2) + diff(:,:,3);
    diff_2_norm = sqrt(diff(:,:,2).^2 + diff(:,:,3).^2);

        

%     figure(hh), hist(diff_2_norm(:), 100)
    % rgb thresh.
%     mask = diff_1norm > kRgbThreshold;

    % hue thresh.
%     mask = diff(:,:,1) > kHueThresh;
    mask1 = (diff_2_norm > kUvThresh);% | (h < .09 & diff_2_norm > 1);
%     mask1 = mask1 | diff(:,:,1) > 30;
%     mask1 = diff_1norm > kHueThres1h;
    mask2 = (r > 0.9 * g) & (g > 0.6 * b) & (r > .9 * b) & ...
            (h < 0.8 * s) & (h > 0.01) & (h < 0.2);
    
    mask1 = mask1 & mask2;
    mask = RegionGrow(mask1, im_show);
    
    if DEBUG_IMAGES
        r = im_show(:,:,1);
        r(mask) = 255;
        im_show(:,:,1) = r; 

        figure(mask_fig);
        imshow(im_show);
        if SAVE_DEBUG 
            debug_file = sprintf(debug_template, i);
%             imwrite(im_show, debug_file);
        end
    end
%     if i == 369
%         pause;
%     end
    [train_im, train_mask] = CropROI(im_show, mask, output_height, ...
                                     output_width);
    if isempty(train_im) || isempty(train_mask)
        continue;
    end
%     imwrite(train_im, hand_file);
%     imwrite(train_mask, mask_file);
%     disp(['saving: ' num2str(i)]);
end

function out_mask = RegionGrow(in_mask, original_image)
    out_mask = in_mask;
end

function [lighting_gain] = ransac_background_diff(bg_ycbcr, im_ycbcr)
    [im_m, im_n, ~] = size(im_ycbcr);
    [bg_m, bg_n, ~] = size(bg_ycbcr);
    
    assert(im_m == bg_m);
    assert(im_n == bg_n);
    
    diff = im_ycbcr - bg_ycbcr;
    
    kDiffInflation = 1.1;
    kInlierThresh = 1;
%     kInflationRate = 0.1;  %  Allow 10% difference.
    inlier_count = 0;
    optimal_lighting_diff = [0 0 0];
    for i = 1 : 100
        xy = 1 + ([bg_n+1, bg_m+1] - 1).*rand(1, 2);
        x = floor(xy(1));
        y = floor(xy(2));
        random_sample = diff(y, x, :);
        
        diff_minus_sample = diff - random_sample .* kDiffInflation;
        
        count = sum(sum(sum(diff_minus_sample .* diff_minus_sample, 3) <= 1));
        
        
        if count > inlier_count
            inlier_count = count;
            optimal_lighting_diff = random_sample;
        end
    end
    lighting_gain = optimal_lighting_diff;
end

