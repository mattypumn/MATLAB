landmarkpath = "/usr/local/google/home/mpoulter/for_matt/height_out/MinDist_SubPix/landmark_counts_ds3_mindist5.txt";

info = dlmread(landmarkpath);

disp(['samples: ' num2str(size(info,1))]);
disp(['total: ' num2str(sum(info))]);
disp(['average: ' num2str(mean(info))]);
disp(['std.dev: ' num2str(std(info))]);
disp(['min: ' num2str(min(info))]);
disp(['max: ' num2str(max(info))]);
