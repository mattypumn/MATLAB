

input_directory = '~/for_matt/pixel_finger/static_camera/exp2/dump/feature_tracking_cropped';
output_directory = '~/for_matt/pixel_finger/static_camera/exp2/dump/feature_tracking_cropped_png';
input_format = 'image_%05d.ppm';
output_format = 'image_%05d.png';

system(['mkdir -p ' output_directory]);

for im_num = 1 : 100000
    input_file = fullfile(input_directory, num2str(im_num, input_format));
    output_file = fullfile(output_directory, num2str(im_num, output_format));
    im = imread(input_file);
    imwrite(im, output_file);
    disp(['Saved ' num2str(im_num, output_format)]);
end

