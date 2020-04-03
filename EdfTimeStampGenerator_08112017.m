function EdfTimeStampGenerator_08112017
% This program creates a time stamp file to be used in Sleepscorer.
%% Select EDF file to generate time stamp file:
working_dir = pwd;
current_dir = 'C:\'; % This is the default directory that opens.
cd(current_dir);
[filename, pathname] = uigetfile('*.edf', 'Select an EDF file'); % Waits for user input and limits selection to .edf files.
% Check for whether or not a file was selected
if isempty(filename) || isempty(pathname)
    uiwait(errordlg('You need to select an EDF file. Please try again',...
        'ERROR','modal'));
    cd(working_dir);
else
    cd(pathname);
    edfFile= fullfile(pathname, filename);    
end

%% Load the data needed to create the time stamp file:
[header, ~, ~] = blockEdfLoad(edfFile);
lengthInSeconds = header.data_record_duration * header.num_data_records;
num2HrBlocks = ceil(lengthInSeconds/(2 * 60 * 60));
outputData = zeros(num2HrBlocks, 6);
for i = 1:num2HrBlocks-1
    startTime = (i-1) * 7200;
    stopTime = i * 7200 - 1;
    outputData(i, :) = [startTime stopTime 1 7200 startTime stopTime];
end
startTime = (num2HrBlocks-1) * 7200;
stopTime = lengthInSeconds - 1;
exactStop = 7200 * (lengthInSeconds/7200 - floor(lengthInSeconds/7200));
outputData(num2HrBlocks, :) = [startTime stopTime 1 exactStop startTime stopTime];

%% Request user input to name time stamp file:
prompt = {'Enter the filename you want to save it as: (just the name)'};
def = {'SubjectNumberDate'};
dlgTitle = 'Input for Timestamp utility';
lineNo = 1;
answer = inputdlg(prompt,dlgTitle,lineNo,def);
filename = char(answer(1,:));
timestampfile = strcat(filename,'.xls');

%% Write to Excel (1997-2003 .XLS) file (can be <2hr):
xlswrite(timestampfile, outputData, 'Sheet1');
cd(working_dir);