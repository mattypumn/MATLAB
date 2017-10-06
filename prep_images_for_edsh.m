%% Parameters.
images = {};
masks = {};
labels = {};
images{1} = '~/for_matt/EDSH/edsh1/imgs';
images{2} = '~/for_matt/EDSH/edsh2/imgs';
images{3} = '~/for_matt/EDSH/edshk/imgs';
masks{1} = '~/for_matt/EDSH/edsh1/masks';
masks{2} = '~/for_matt/EDSH/edsh2/masks';
masks{3} = '~/for_matt/EDSH/edshk/masks';
labels{1} = 'edsh1';
labels{2} = 'edsh2';
labels{3} = 'edshk';

image_template = 'image_%06d.jpg';
save_template = '%08d.jpg';

output_base = '~/for_matt/EDSH/training';

%% Setup.
mask_out_dir = fullfile(output_base, 'mask');
img_out_dir = fullfile(output_base, 'img');
% mkdir_command = ['mkdir -p ' output_dir];
% system(mkdir_command);

image_counter = 1;
for dir_i = 1 : length(images)
    %% Get proper information.
    masks_dir = masks{dir_i};
    images_dir = images{dir_i};
    label = labels{dir_i};
    mask_template = [upper(label), '_mask%d.jpg'];
    
    %% Walk through directory.
    mask_files = dir(masks_dir);
    for file_i = 1 : length(mask_files)
        %% Get one file.
        m_file = mask_files(file_i);
        %% Load if it is an image.
        if ~m_file.isdir 
            image_number = sscanf(m_file.name, mask_template);
            disp([num2str(image_counter), ' ', ...
                    label, ' ' num2str(image_number) ]);
            mask_file = fullfile(masks_dir, m_file.name);
            image_file = fullfile(images_dir, ...
                         sprintf(image_template, image_number));
            if exist(image_file, 'file') == 2
                try 
                    im_rgb = imread(image_file);
                    im_mask = imread(mask_file);
                catch 
                    continue;
                end
                im_mask = imresize(im_mask, [720 1280]);
                im_rgb = imresize(im_rgb, [720 1280]);
                
                outfilename = sprintf(save_template, image_counter);
                
                imwrite(im_mask, fullfile(mask_out_dir, outfilename));
                imwrite(im_rgb, fullfile(img_out_dir, outfilename));
                
                image_counter = image_counter + 1;
            else
               disp(['could not find image file ' image_file]);
               pause;
            end
        end
    end
end