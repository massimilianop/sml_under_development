function [] = print_html(file_name)
%print_html print plots in html file, of data saved in file "filename"
%   Detailed explanation goes here

% example : assignin('base', 'file_name', '_1458747408_test_plot_1.txt');
% arguments must exist in the base workspace
% need to include file_name in base workspace, since 
file_name = file_name(1:end-4);
assignin('base', 'file_name', strcat(file_name,'.txt'));

% include functions
addpath(genpath('../plot_in_matlab'))
addpath(genpath('../plot_in_matlab/functions'))

mkdir(file_name)
opts.outputDir = file_name;

publish('plot_in_matlab.m',opts)

end