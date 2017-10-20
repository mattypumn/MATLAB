function [crop_im, crop_mask] = CropROI(image, mask, roi_height, roi_width)
    assert (size(image, 1) == size(mask, 1));
    assert (size(image, 2) == size(mask, 2));
    assert (roi_height > 1);
    assert (roi_width > 1);
    
    image_height = size(image, 1);
    image_width = size(image, 2);
    
    %% Constants.
    kGrowthPercentage = 0.25;
    
    %% Find tight bound.
    stats = regionprops(mask, 'all');
    n = length (stats);
    max_area = 0;
    bounding_box = [];
    for i = 1 : n
        stat = stats(i);
        area = stat.Area;
        if max_area < area
            max_area = area;
            bounding_box = stat.BoundingBox;
        end
    end
    
    if isempty(bounding_box) || length(bounding_box) ~= 4
        crop_im = [];
        crop_mask = [];
        return;
    end
    
    
    
    %% Increase by 10%.
    width_inc = bounding_box(3) * kGrowthPercentage;
    height_inc = bounding_box(4) * kGrowthPercentage;
    new_upperleft_x = max(1, bounding_box(1) - 0.5 * width_inc);
    new_upperleft_y = max(1, bounding_box(2) - 0.5 * height_inc);
    lowerright_x = min(image_width, ...
            new_upperleft_x + bounding_box(3) + 0.5 * width_inc);
    lowerright_y = min(image_height, ...
            new_upperleft_y + bounding_box(4) + 0.5 * height_inc);

    new_box = round([new_upperleft_x, new_upperleft_y, ...
                          lowerright_x - new_upperleft_x - 1, ...
                          lowerright_y - new_upperleft_y - 1]);
 
      
%     imshow(image);
%     top_left = new_box(1:2);
%     bottom_right = new_box(1:2) + new_box(3:4);
%     
%     
%     rectangle('Position', [top_left, bottom_right],...
%                 'EdgeColor','r', 'LineWidth', 1);
      
      
      
      
      
    %% Grow for proper ratio.
    new_box2 = GrowBoundingBoxToRatio(new_box, roi_height, roi_width, ...
                                     [image_height, image_width]);
   
    %% Crop the image.
    crop_im = image(new_box2(2):new_box2(2)+new_box2(4), ...
                       new_box2(1):new_box2(1)+new_box2(3), :);
    
    crop_mask = mask(new_box2(2):new_box2(2)+new_box2(4), ...
                        new_box2(1):new_box2(1)+new_box2(3), :);
                    
    %% Resize the image to fit the roi height and width.
    crop_im = imresize(crop_im, [roi_height, roi_width]);
    crop_mask = imresize(crop_mask, [roi_height, roi_width]);
end

function box = GrowBoundingBoxToRatio(BoundingBox, roi_height, ...
                                      roi_width, image_size)
    % BoundingBox [x, y, width, height]
    start_x = BoundingBox(1);
    start_y = BoundingBox(2);
    start_w = BoundingBox(3);
    start_h = BoundingBox(4);
    target_ratio = roi_height / roi_width;
    current_ratio = start_h / start_w;
    
    if current_ratio == target_ratio
        box = BoundingBox;
        return;
    end
        
    new_upperleft_x = [];
    new_upperleft_y = [];
    new_h = [];
    new_w = [];
    
    if current_ratio < target_ratio 
        % Need to grow height.
        target_height = start_w * target_ratio;
        diff_h = target_height - start_h;
        move_dist = 0.5 * diff_h;
        %% Move upper boundary of box.
        new_upperleft_y = start_y - move_dist;
        if new_upperleft_y < 1
            extra = 1 - new_upperleft_y;
            diff_h = diff_h + extra;
            new_upperleft_y = 1;
        end

        %% Move lower boundary of box.
        new_h = start_h + diff_h;
        if new_upperleft_y + new_h > image_size(1)
            %% TODO(mpoulter) Shrink width. 
            % Currently not implemented as we do not want to lose and 
            % of the mask. 
            new_h = floor(image_size(1) - new_upperleft_y);
        end
        new_upperleft_x = start_x;
        new_w = start_w;
    else 
        % need to grow width.
        target_width = start_h / target_ratio;
        diff_w = target_width - start_w;
        move_dist = 0.5 * diff_w;
        %% Move left boundary of box.
        new_upperleft_x = start_x - move_dist;
        if new_upperleft_x < 1
            extra = 1 - new_upperleft_x;
            diff_w = diff_w + extra;
            new_upperleft_x = 1;
        end

        %% Move lower boundary of box.
        new_w = start_w + diff_w;
        if new_upperleft_x + new_w > image_size(2)
            %% TODO(mpoulter) Shrink width. 
            % Currently not implemented as we do not want to lose and 
            % of the mask. 
            new_w = floor(image_size(2) - new_upper_left_x);
        end
        new_upperleft_y = start_y;
        new_h = start_h;
    end
    box = round([new_upperleft_x, new_upperleft_y, new_w, new_h]);
end



