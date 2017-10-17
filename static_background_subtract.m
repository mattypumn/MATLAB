clear; clc; close all;


dataset = '~/for_matt/pixel_finger/static_camera/exp7';
images_dir = fullfile(dataset, 'dump', 'feature_tracking_cropped');
template = 'image_%05d.ppm';

kNumSuperPixels = 500;
kSuperPixelMinArea = 0.75;

kRgbThreshold = 10;
kHueThresh = 0.1;
kSatThresh = .01;
kUvThresh = 8;


read_template = fullfile(images_dir, template);
bg_image_num = 40;
first_image = 250;
last_image = 3000;


bg_im = imread(sprintf(read_template, bg_image_num));
bg_im_show = bg_im;
% bg_im = rgb2hsv(bg_im);
% bg_im(:,:,1) = medfilt2(bg_im(:,:,1));
% bg_im(:,:,2) = medfilt2(bg_im(:,:,2));
% bg_im(:,:,3) = medfilt2(bg_im(:,:,3));

bg_im = rgb2ycbcr(double(bg_im));
bg_im(:,:,1) = medfilt2(bg_im(:,:,1));
bg_im(:,:,2) = medfilt2(bg_im(:,:,2));
bg_im(:,:,3) = medfilt2(bg_im(:,:,3));

imshow(bg_im_show);

fig = figure();
sp_fig = figure();
% hh = figure();
for i = first_image : last_image
    read_file = sprintf(read_template, i);
    im = imread(read_file);
    
    
    
    
    % todo medfilt2
    r = medfilt2(im(:,:,1));
    g = medfilt2(im(:,:,2));
    b = medfilt2(im(:,:,3));
      
    im_show = im;
    im_hsv = rgb2hsv(double(im));
    im_hsv(:,:,1) = medfilt2(im_hsv(:,:,1));
    im_hsv(:,:,2) = medfilt2(im_hsv(:,:,2));
    im_hsv(:,:,3) = medfilt2(im_hsv(:,:,3));
    h = im_hsv(:,:,1);
    s = im_hsv(:,:,2);
    v = im_hsv(:,:,3);
    im = rgb2ycbcr(double(im));
    im(:,:,1) = medfilt2(im(:,:,1));
    im(:,:,2) = medfilt2(im(:,:,2));
    im(:,:,3) = medfilt2(im(:,:,3));
    



    diff = abs(bg_im - im);
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

    r = im_show(:,:,1);
    r(mask) = 255;
    im_show(:,:,1) = r; 
    
    figure(fig);
    imshow(im_show);
%     pause(1)
end



function out_mask = RegionGrow(in_mask, original_image)
    out_mask = in_mask;
end


