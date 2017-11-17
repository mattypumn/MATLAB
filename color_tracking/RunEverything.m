clear all;
close all;

addpath('.\Static');
addpath('.\Dynamic');
addpath('.\Data');

DATA_PATH = '..\Data\';
RES_PATH = '..\Results\';

for I = 1:21
    result_path{I} = sprintf('%s%s%d%s', RES_PATH, 'sequence', I,'\');
    file_path{I} = sprintf('%s%s%d%s', DATA_PATH, 'sequence', I,'\');
    file_base{I} = 'frame';
end;

num_comp = [1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,3,4];

frames{1} = 0:99;
frames{2} = 0:71;
frames{3} = 0:71;  
frames{4} = 0:109;    
frames{5} = 0:74;   
frames{6} = 0:71;    
frames{7} = 0:75;   
frames{8} = 0:72;    
frames{9} = 0:71;    
frames{10} = 0:72;   
frames{11} = 0:232;    
frames{12} = 0:71;   
frames{13} = 0:349;
frames{14} = 0:71;    
frames{15} = 0:74;    
frames{16} = 0:49;    
frames{17} = 0:74;    
frames{18} = 0:90;   
frames{19} = 0:506;    
frames{20} = 0:72;   
frames{21} = 0:119;   
frames{22} = 0:52;

res = [];

stat_results = {};
dyn_results = {};

for I = 1:21
  clear global;
  
  global SHOW;
  SHOW = 1;
  
  global SAVE_MASK;
  SAVE_MASK = strcat(file_path{I}, 'static');
  StaticSkinTracker(num_comp(I), file_path{I}, file_base{I}, frames{I}); 

  global SAVE_MASK;
  SAVE_MASK = strcat(file_path{I}, 'dynamic');
  ColorTracker(num_comp(I), file_path{I}, file_base{I}, frames{I});  
end;

