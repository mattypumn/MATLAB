clear all;
close all;

addpath('mex');
vxx = zeros(480, 640, 50);
vyy = zeros(480, 640, 50);

for n = 10:59

example = 'pixel_dataset/';
im1 = im2double(imread([example sprintf('image_%05d',n) '.pgm']));
im2 = im2double(imread([example sprintf('image_%05d',n+1) '.pgm']));

% set optical flow parameters (see Coarse2FineTwoFrames.m for the definition of the parameters)
alpha = 0.012;
ratio = 0.75;
minWidth = 20;
nOuterFPIterations = 7;
nInnerFPIterations = 1;
nSORIterations = 30;

para = [alpha,ratio,minWidth,nOuterFPIterations,nInnerFPIterations,nSORIterations];

% this is the core part of calling the mexed dll file for computing optical flow
% it also returns the time that is needed for two-frame estimation
tic;
[vx,vy,warpI2] = Coarse2FineTwoFrames(im1,im2,para);
toc

vxx(:,:, n-9) = vx;
vyy(:,:, n-9) = vy;

% figure;
% subplot(121),imshow(im1);subplot(122),imshow(warpI2);

% output gif
% clear volume;
% volume(:,:,:,1) = im1;
% volume(:,:,:,2) = im2;
% if exist('output','dir')~=7
%     mkdir('output');
% end
% frame2gif(volume,fullfile('output',[example '_input.gif']));
% volume(:,:,:,2) = warpI2;
% frame2gif(volume,fullfile('output',[example '_warp.gif']));


% visualize flow field
% clear flow;
% flow(:,:,1) = vx;
% flow(:,:,2) = vy;
% imflow = flowToColor(flow);
% figure;imshow(imflow);
% 
% figure;
% subplot(121),imagesc(vx);
% subplot(122),imagesc(vy);

% figure, 
% e1 = edge(im1);
% vx_ = vx; vx_(e1 == 0) = 0;
% vy_ = vy; vy_(e1 == 0) = 0;
% vv = sqrt(vx_.*vx_ + vy_.*vy_);
% [y, x] = find(vv > 0);
% vx_ = vx_(vv>0);
% vy_ = vy_(vv>0);
% 
% quiver(x, 480-y, vx_, vy_, 1.5)
% axis([1 640 1 480])
end

save flow vxx vyy

