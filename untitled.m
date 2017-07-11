data_dir = '/usr/local/google/home/mpoulter/for_matt/bags_only/data1/dump/points/'


file = [data_dir 'pts_35.asc'];

data = dlmread(file);

plot3(data(:,1), data(:,2) , data(:,3),'.');
axis equal;




       x = (trace([  0.999631 -0.0195312  0.0188591;
 0.0163242    0.98741   0.157336;
-0.0216947   -0.15697   0.987365
]) - 1) / 2;


