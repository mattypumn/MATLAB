%%% This script has been created to download datasets from gcloud
%%% In order to process them on a a local machine. 
%%% The majority of the meta data was retrieved from
%%% https://drive.google.com/open?id=0BxxzyyjF8Sh5Y1o3WGdab1dfam8
%%% which gives the directory on gcloud as well as the information
%%% for the device being used.


%% Parameters.
data_descriptions_file = '/usr/local/google/home/mpoulter/for_matt/average_calibrations/S8_data_9_24.txt';
letango_dir = '~/for_matt/letango_collection';
model_dir = '/usr/local/google/home/mpoulter/for_matt/average_calibrations';

%% Read metadata.
fid = fopen(data_descriptions_file);
data_descriptions = textscan(fid, '%s');
fclose(fid);
data_cells = data_descriptions{1,1};

%% For each dataset
for i = 1 : length(data_cells)
    %% Format.
    line = data_cells{i};
    line_cells = strsplit(line,',');
     
    %% Get gcloud dir.
    gcloud_dir = line_cells{end};
    gcloud_dir_cells = strsplit(gcloud_dir, '/');
    data_name = gcloud_dir_cells{end};
    data_id = gcloud_dir_cells{end-1};
    new_data_name = [data_id '_' data_name];
    
%     if exist( fullfile(letango_dir, new_data_name), 'dir')
%         continue;
%     end    
    
    %% Download data.
%     dl_command = ['gsutil -m cp -r' ' ' gcloud_dir ' ' letango_dir];
%     retval = -1;
%     safety = 0;
%     disp(['downloading: ' gcloud_dir]);
%     while retval ~= 0 && safety < 5
%         [retval, stdout] = system(dl_command);
%         safety = safety + 1;
%         if retval ~= 0 
%             disp(stdout);
%         end
%     end
%     
%     if safety >= 5
%        disp(['Error downloading dataset: ' cloud_dir]);
%        continue;
%     end
    
    %% Rename folder to prevent collisions.
%     mv_command = ['mv' ' ' fullfile(letango_dir, data_name) ' ' fullfile(letango_dir, new_data_name)];
%     retval = system(mv_command);
%     
%     if retval ~= 0
%         disp(['Error moving file: ' mv_command]);
%         pause;
%     end
%     disp('Completed Download');
    
    
     %% Save average calibration within dataset.
%     model = line_cells{3};
%     model = strrep(model, '/', '_');
%     model_calibration = fullfile(model_dir, [model '.xml']);
%     model_command = ['cat' ' ' model_calibration ' > ' fullfile(letango_dir, new_data_name, 'average_calibration.xml')];
%     system(model_command);
    
end