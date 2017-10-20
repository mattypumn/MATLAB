height = 416; width = 640;

%% Parameters.
base_dir = '~/for_matt/pixel_finger/color/exp1';

%% Setup.
marker_file = fullfile(base_dir, marker_pixels.txt);
image_dir = fullfile(base_dir, 'feature_tracking_primary');


pts = dlmread(marker_file);
markers = pts(:,2:7);
marker_x = markers(:,[1 3 5]);
marker_y = markers(:,[2 4 6]);

valid_idx = (sum((marker_x >= 1) & (marker_x <= width) & (marker_y >= 1) & (marker_y <= height), 2) == 3);

for i=0:(length(valid_idx) - 1)
    if (~valid_idx(i+1))
        continue;
    end
%     disp([i, markers(i+1,:)])
    
    name = sprintf('/usr/local/google/home/mrinalkanti/Downloads/Experiment 5/feature_tracking_primary/image_%05d.ppm', i);
    im = imread(name);
    
    im_cropped = im(1:height,:,:);
    r = double(im_cropped(:,:,1));
    g = double(im_cropped(:,:,2));
    b = double(im_cropped(:,:,3));
    r = medfilt2(r);
    g = medfilt2(g);
    b = medfilt2(b);
    
    im_hsv = rgb2hsv(im_cropped);
    h = im_hsv(:,:,1);
    s = im_hsv(:,:,2);
    v = im_hsv(:,:,3);
    h = medfilt2(h);
    s = medfilt2(s);
    v = medfilt2(v);
    
    mask = (r > 1.1 * g) & (g > 0.9* b) & (r > 1.2 * b) & (h < 0.35 * s) & (h > 0.03) & (h < 0.09);
    mask = bwareafilt(logical(mask),1,'largest');
%     mask = imfill(mask,'holes');
    idx = find(mask);
    
    im_disp = im_cropped;
    disp_r = im_cropped(:,:,1); disp_r(idx) = 255;
    im_disp(:,:,1) = disp_r;
    imshow(im_disp)
%     return;
    pause(.01);
end
