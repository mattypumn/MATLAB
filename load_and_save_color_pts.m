filename_in = '/usr/local/google/home/mpoulter/Desktop/global_cloud.asc';
filename_out = '/usr/local/google/home/mpoulter/Desktop/global_cloud.ply';
data = dlmread(filename_in);

data = [data, data(:,end), data(:,end)];
save_ply(filename_out, data);