for i =20:50
    vx = vxx(:,:,i);
    vy = vyy(:,:,i);
idx = rand(480, 640);
e1 = idx > .995;
vx_ = vx; vx_(e1 == 0) = 0;
vy_ = vy; vy_(e1 == 0) = 0;
vv = sqrt(vx_.*vx_ + vy_.*vy_);
[y, x] = find(vv > 0);
vx_ = vx_(vv>0);
vy_ = vy_(vv>0);

example = 'pixel_dataset/';
im1 = im2double(imread([example sprintf('image_%05d',i+9) '.pgm']));
subplot(211), imshow(im1)
subplot(212),quiver(x, 480-y, vx_, vy_, 1.1), axis([1 640 1 480]), axis equal;
% return;
pause(.02)
end