%%  Parameters.
dataset = fullfile('pixel_finger', 'color', 'exp6');

%% Setup.
input_directory = fullfile('~/for_matt', dataset, 'dump', ...
    'feature_tracking_cropped');
in_format = 'image_%05d.ppm';

output_directory = fullfile('~/for_matt', dataset, 'dump', ...
    'feature_tracking_cropped_png');
out_format = 'image_%05d.png';



system(['mkdir -p ' output_directory]);

for i = 0 : 10000
    image_file = fullfile(input_directory, sprintf(in_format, i));
    if ~(exist(image_file, 'file') == 2)
%         disp(['Skipping: ' num2str(i, out_format)]);
        continue;
    end
    out_file = fullfile(output_directory, sprintf(out_format, i));
    im = imread(image_file);
    imwrite(im, out_file);
    disp(['Saved: ' num2str(i, out_format)]);
end
