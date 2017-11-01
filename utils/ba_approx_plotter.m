clc;
clear;

base_path = '~/ba_approx/test_approx/old-results';
% base_path = '~/ba_approx/test_approx/';


% dataset_path = fullfile(base_path, '/dataset-1/');
% wnd_start = 500;
% wnd_end = 1705;

% dataset_path = fullfile(base_path, '/dataset-6/');
% wnd_start = 165;
% wnd_end = 235;

% dataset_path = fullfile(base_path, '/yellowstone-lab-2/');
% wnd_start = 103;
% wnd_end = 250;

dataset_path = fullfile(base_path, '/yellowstone-walter-circle/');
wnd_start = 589;
wnd_end = 1750;


xkk1 = dlmread(fullfile(dataset_path, 'ba_approx_method1/xkk.txt'));
xkk2 = dlmread(fullfile(dataset_path, 'ba_approx_method2/xkk.txt'));
xkk3 = dlmread(fullfile(dataset_path, 'ba_approx_method3/xkk.txt'));
xkk4 = dlmread(fullfile(dataset_path, 'ba_approx_method4/xkk.txt'));
xkk_ba = dlmread(fullfile(dataset_path, 'ba_from_logged_data/xkk.txt'));

N = size(xkk_ba, 2);
range = 1:N;

range1 = 1:wnd_start;
range2 = wnd_end:N;

range3 = [range1, range2];

PlotPoints(xkk_ba(14:16, range), '-k');
%PlotPoints(xkk_ba(14:16, range2), '-k');
hold on;
% PlotPoints(rotz(1*pi/180) * xkk2(14:16, range), '-r');
%PlotPoints(xkk2(14:16, range2), '-r');
hold on;
PlotPoints(rotz(2*pi/180) *xkk3(14:16, range), '-m');
% %PlotPoints(xkk3(14:16, range2), '-m');
% hold on;
% PlotPoints(xkk4(14:16, range), '-c');
% %PlotPoints(xkk4(14:16, range2), '-c');
% 
% PlotPoints(xkk1(14:16, range1), '-b');
% PlotPoints(xkk1(14:16, range2), '-b');

%legend('BLS', 'C-KLAM', 'Duplicate Poses', 'DFM', 'Remove Poses');
view([90 90]);
