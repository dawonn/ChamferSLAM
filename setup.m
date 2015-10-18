% setup.m Loads a dataset and sets dataset-specific parameters

% Clear the workspace
clear

% Set up paths to utility functions
wd = pwd;
addpath( wd, [wd '/util']);

% Output Path
OutPath = '../output/';
mkdir(OutPath);

% Datasets Path
DataPath = '~/datasets/';

% Dataset default ROI (Scan Indexes)
start = 1;
stop  = 10e7;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Select dataset to run
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 DatasetName = 'hall_and_room_w_vn';       
% DatasetName = 'eerc_dillman_dow';         
% DatasetName = 'campus1';                   
% DatasetName = 'eerc_dow_dill_inout';  
      
% DatasetName = '2000-01-31-19-01-35';  % EERC817 Multiple passes
% DatasetName = '2000-01-31-19-05-32';  % EERC8F
% DatasetName = '2000-01-31-19-10-14';  % EERC1F
% DatasetName = '2000-01-31-19-13-51';  % EERC1F small loop   
% DatasetName = '2000-01-31-19-15-35';  % EERC1F large loop
% DatasetName = '2000-01-31-19-21-26';  % EERC_DOW_DIL inout 
% DatasetName = '2000-01-31-19-50-23';  % Campus 
% DatasetName = '2000-01-31-20-30-59';  % EERC8f elevator to ieee, 817
         
% DatasetName = 'EERC8f handheld';  % EERC 8f IEEE, Lap, stairwells. (good)
% DatasetName = 'EERC_DOW_DIL inout3';  %  handheld {1, 3k, 11k, 24k }

% Night with Natalie #2 (EERC 8F)
%DatasetName = '2015-04-17-01-30-48';  
%start       = 50;
            
% Night with Natalie #1 (EERC 8F)
%DatasetName = '2015-04-17-00-44-23';  
%stop        = 13500;
            
%DatasetName = 'vicon1';
% EERC 8f
%stop = 2800;

% EERC 5f
%start = 2900;
%stop  = 7500;

% Vicon 1
%DatasetName = 'vicon1';
%start = 45000;
%stop  = 60000;

%DatasetName = 'vicon2';
%DatasetName = 'vicon3';
%DatasetName = 'vicon4';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Load dataset from log files
ReadLogfiles


% Ready to run SLAM algorithm
SLAM


% Future runs of this dataset can call SLAM directly untill a new dataset
% is required.
