%% Consts.
do_output_binary_mask = true;
roi_height = 240;
roi_width = 240;

%%  Parameters.
dataset = fullfile('pixel_finger', 'static_camera', 'exp1');
input_image_directory = fullfile('~/for_matt', dataset, 'dump', ...
    'feature_tracking_cropped_png');
input_mask_directory = fullfile('~/for_matt', dataset, 'dump', ...
    'full_masks');
input_mask_format = 'mask_image_%05d.png';
input_image_format = 'image_%05d.png';

output_directory = fullfile('~/for_matt', dataset, 'dump', ...
    'cropped_hand');
output_image_format = 'image_%05d.png';
output_mask_format = 'mask_%05d.png';

%% Setup.
system(['mkdir -p ' output_directory]);

%% Main loop.
for i = 0 : 10000
    %% Format files.
    input_image_file = fullfile(input_image_directory, ...
        sprintf(input_image_format, i));
    input_mask_file = fullfile(input_mask_directory, ...
        sprintf(input_mask_format, i));
    output_image_file = fullfile(output_directory, ...
        sprintf(output_image_format, i));
    output_mask_file = fullfile(output_directory, ...
        sprintf(output_mask_format, i));

    %%  If mask is not found, no problem. Go to the next.
    if ~(exist(input_mask_file, 'file') == 2)
        continue;
    end
    
    %% If mask exists and image does not, break.
    if ~(exist(input_image_file, 'file') == 2)
        disp(['ERROR: Could not find matching image. ' input_image_file]);
        return;
    end
    
    %% Load original.
    image = imread(input_image_file);
    mask = imread(input_mask_file);
    
    %% Mak mask Binary.
    if do_output_binary_mask
        binary_mask = (mask(:,:,1) > 5) | (mask(:,:,2) > 5) | ...
            (mask(:,:,3) > 5);
    end
    
    %% Crop image and mask.
    [cropped_im, cropped_mask] = CropROI(image, binary_mask, ...
        roi_height, roi_width);
    
    if isempty(cropped_im) || isempty(cropped_mask)
        continue;
    end
       
    %% Save pair for trainig.
    imwrite(cropped_im, output_image_file);
    disp(['Saved: ' sprintf(output_image_format, i)]);
    imwrite(cropped_mask, output_mask_file);    
    disp(['Saved: ' sprintf(output_mask_format, i)]);
end

%% Finished !
disp('Finished!');
