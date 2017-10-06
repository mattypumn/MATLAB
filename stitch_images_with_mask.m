%% Parameters.
data_dir = '~/for_matt/EDSH/';
label = 'edshk';
mask_template = [upper(label), '_mask%d.jpg'];
image_template = 'image_%06d.jpg';

%% Setup.
images_dir = fullfile(data_dir, label, 'imgs');
masks_dir = fullfile(data_dir, label, 'masks');
output_dir = fullfile(data_dir, label, 'stitched');
mkdir_command = ['mkdir -p ' output_dir];
system(mkdir_command);

mask_files = dir(masks_dir);
for i = 1 : length(mask_files)
    m_file = mask_files(i);
    if ~m_file.isdir 
        image_number = sscanf(m_file.name, mask_template);
        disp(num2str(image_number));
        mask_file = fullfile(masks_dir, m_file.name);
        image_file = fullfile(images_dir, ...
                     sprintf(image_template, image_number));
        disp(mask_file);
        disp(image_file);
        if exist(image_file, 'file') == 2
            out_file = fullfile(output_dir, ...
                            sprintf('stitched_%06d.jpg', image_number));
            try 
                im_rgb = imread(image_file);
                im_mask = imread(mask_file);
            catch 
                continue;
            end

            im_mask = imresize(im_mask, [720 1280]);
            im_rgb = imresize(im_rgb, [720 1280]);
            
            im_mask(:,:,2) = im_mask(:,:,1);
            im_mask(:,:,3) = im_mask(:,:,1);
            im_mask = im_mask * 255;
            im_merged = [im_rgb, im_mask];
            imwrite(im_merged, out_file);
        else 
           disp(['could not find image file ' image_file]);
           pause;
        end
    end
end