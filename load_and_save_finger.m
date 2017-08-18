%% Parameters.
joint_data_file = '~/for_matt/nyu_hand/train/joint_data.mat';
out_dir = '~/for_matt/nyu_hand/train/rgb1_hot_vectors';

%%  Setup.
CAMERA_ID = 1;
FINGER_POINT_ID = 19;
IMAGE_WIDTH = 640;
IMAGE_HEIGHT = 480;

%% Main Loop.
data = load(joint_data_file);
uvd = data.joint_uvd;
file_map = [];
for i = 1 : size(uvd, 2)
    u = uvd(CAMERA_ID, i, FINGER_POINT_ID, 1);
    v = uvd(CAMERA_ID, i, FINGER_POINT_ID, 2);
    file_map = [file_map; i u v];
end

dlmwrite([out_dir '/camera_' num2str(CAMERA_ID) '_imageUVmap_indextip.txt'],file_map, ' ' );