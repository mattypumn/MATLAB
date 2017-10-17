height = 416; width = 640;

pts = dlmread('/usr/local/google/home/mrinalkanti/Downloads/Experiment 5/marker_pixels.txt');
markers = pts(:,2:7);
marker_x = markers(:,[1 3 5]);
marker_y = markers(:,[2 4 6]);

valid_idx = (sum((marker_x >= 1) & (marker_x <= width) & (marker_y >= 1) & (marker_y <= height), 2) == 3);

vals = [];
temp = repmat((1:height)', 1, width)';
xy_im = [repmat(1:width, 1, height)', temp(:)];
radius = 10;

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
    
    xy = [marker_x(i+1,:)', marker_y(i+1,:)'];
    xy_all = [];
    for kk = 1:3
        xy_k = xy(kk,:);
        xy_all = [xy_all; xy_im(sqrt(sum((xy_im-repmat(xy_k, length(xy_im), 1)).^2,2)) < radius, :)];
    end
    idx = sub2ind(size(im_cropped), xy_all(:,2), xy_all(:,1));
    
    vals = [vals; [r(idx) g(idx) b(idx) h(idx) s(idx) v(idx)]];
    
%     imshow(im_cropped), hold on, plot(marker_x(i+1,:), marker_y(i+1,:),'x')
%     viscircles(xy, 10*ones(3,1));
%     pause;
end

tt = sort(vals(:,4));
tt(round(length(tt)*.1))
tt(round(length(tt)*.9))