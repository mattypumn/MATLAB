function [ timestamps ] = ReadImageTimestampFile( timestamp_file )
%READIMAGETIMESTAMPFILE Summary of this function goes here
%   Detailed explanation goes here
    
% Read image data.
fid = fopen( timestamp_file );
tline = fgetl( fid );
timestamps = [];
while( ischar(tline) )
    line_cell = strsplit(tline, ' ');
    num_str = line_cell{1};
    time_str = line_cell{2};
    img_num = str2double(num_str(7:11));
    img_time = str2double(time_str);
    timestamps = [timestamps; img_num, img_time];
    
    tline = fgetl(fid);
end
fclose(fid);
end

