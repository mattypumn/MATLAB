dataset = 2;
dataset_base_dir = '/usr/local/google/home/mpoulter/for_matt/polaris_floor_vicon/mrinal/';
images_file = [dataset_base_dir '/' num2str(dataset) '/' 'out/images.txt'];
tango_pose_file = [dataset_base_dir '/' num2str(dataset) '/' 'out/tango_poses.txt'];



% Read tango_pose data.
tango_pose_data = dlmread(tango_pose_file);

% Read image data.
fid = fopen(images_file);
tline = fgetl(fid);
images_file_data = [];
while( ischar(tline))
    line_arr = strsplit(tline, ',');
    new_data = str2double(line_arr(4:end));
    images_file_data = [images_file_data; new_data]; 
    
    tline = fgetl(fid);
end
fclose(fid);

%  Check the angle difference between image_data and tango_data.
ang_diff = [];
for i = 1 : size(tango_pose_data,1)
    tango_rot = quat2rot(tango_pose_data(i, 5:end)');

    image_rot = [images_file_data(i,1:3);
                 images_file_data(i,5:7);
                 images_file_data(i,9:11)];
    test_rot = tango_rot * image_rot'; % should be identity.
    x = (trace(test_rot) - 1) / 2;
    if abs(x) > 1 
       x = 1 * sign(x); 
    end
    theta = acos( x);
    ang_diff = [ang_diff theta];
end



plot( ang_diff);



