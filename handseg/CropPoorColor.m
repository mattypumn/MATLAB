%%  Parameters.
do_output_binary_mask = true;

dataset = fullfile('pixel_finger', 'color', 'exp7');
input_directory = fullfile('~/for_matt', dataset, 'dump', ...
    'feature_tracking_primary');
input_format = 'image_%05d.ppm';

output_directory = fullfile('~/for_matt', dataset, 'dump', ...
    'feature_tracking_cropped');
output_format = 'image_%05d.ppm';

row_start = 1;
row_end = 416;

%% Setup.
system(['mkdir -p ' output_directory]);

for i = 0 : 10000
    input_file = fullfile(input_directory, sprintf(input_format, i));
    if ~(exist(input_file, 'file') == 2)
        continue;
    end
    output_file = fullfile(output_directory, sprintf(output_format, i));
    im = imread(input_file);
    
    im_save =  im(row_start:row_end, :, :);
    imwrite(im_save, output_file);
    disp(['Saving: ' sprintf(output_format, i)]);
end
