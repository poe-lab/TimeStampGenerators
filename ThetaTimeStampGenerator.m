function varargout = ThetaTimeStampGenerator(varargin)
% THETATIMESTAMPGENERATOR MATLAB code for ThetaTimeStampGenerator.fig
%      THETATIMESTAMPGENERATOR, by itself, creates a new THETATIMESTAMPGENERATOR or raises the existing
%      singleton*.
%
%      H = THETATIMESTAMPGENERATOR returns the handle to a new THETATIMESTAMPGENERATOR or the handle to
%      the existing singleton*.
%
%      THETATIMESTAMPGENERATOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in THETATIMESTAMPGENERATOR.M with the given input arguments.
%
%      THETATIMESTAMPGENERATOR('Property','Value',...) creates a new THETATIMESTAMPGENERATOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ThetaTimeStampGenerator_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ThetaTimeStampGenerator_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ThetaTimeStampGenerator

% Last Modified by GUIDE v2.5 01-Mar-2012 00:06:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ThetaTimeStampGenerator_OpeningFcn, ...
                   'gui_OutputFcn',  @ThetaTimeStampGenerator_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before ThetaTimeStampGenerator is made visible.
function ThetaTimeStampGenerator_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ThetaTimeStampGenerator (see VARARGIN)

% Choose default command line output for ThetaTimeStampGenerator
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ThetaTimeStampGenerator wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ThetaTimeStampGenerator_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%##########################################################################
%               CODE FOR SETTING DEFAULT VALUES OF FILTERS
%##########################################################################
% --- Executes on button press in Set_pushbutton.
function Set_pushbutton_Callback(hObject, eventdata, handles) %#ok<*INUSL>
global T_lo T_hi EEG_Fc gapTime

T_lo = 5; set(handles.Theta_lo, 'String', T_lo);
T_hi = 9; set(handles.Theta_hi, 'String', T_hi);

EEG_Fc = 30; set(handles.EEG_cutoff, 'String', EEG_Fc);
gapTime = 0.5;

function tstampsfile_Callback(hObject, eventdata, handles) %#ok<*INUSD>
% hObject    handle to tstampsfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tstampsfile as text
%        str2double(get(hObject,'String')) returns contents of tstampsfile as a double


% --- Executes during object creation, after setting all properties.
function tstampsfile_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function eegfile_Callback(hObject, eventdata, handles)
% hObject    handle to eegfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of eegfile as text
%        str2double(get(hObject,'String')) returns contents of eegfile as a double


% --- Executes during object creation, after setting all properties.
function eegfile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to eegfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% function varargout = fileSelectionMenu_Callback(h, eventdata, handles, varargin)
% global FileFlag
% switch get(handles.fileSelectionMenu,'Value')   
%     case 1
%         FileFlag = 0;
%         errordlg('You must select a file type.','Error');
%     case 2 % Neuralynx
%         FileFlag = 1;
%     case 3 % AD System
%         FileFlag = 2;
%     case 4 % ASCII - Polysmith
%         FileFlag = 3;
%     case 5 % ASCII - EMZA
%         FileFlag = 4;
% end

% --- Executes during object creation, after setting all properties.
% function fileSelectionMenu_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to fileSelectionMenu (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: popupmenu controls usually have a white background on Windows.
% %       See ISPC and COMPUTER.
% if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%     set(hObject,'BackgroundColor','white');
% end



function theta_thresh_Callback(hObject, eventdata, handles)
global T_Threshold
ThetaThreshold = str2double(get(hObject,'String'));   % returns contents of ThetaThreshold as a double
if isnan(ThetaThreshold)
    %set(hObject, 'String', 0);
    errordlg('Input must be a number','Error');
end
T_Threshold = ThetaThreshold; 


% --- Executes during object creation, after setting all properties.
function theta_thresh_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_hi_Callback(hObject, eventdata, handles)
global T_hi
Theta_hi = str2double(get(hObject,'String'));   % returns contents of Theta_hi as a double
if isnan(Theta_hi)
    %set(hObject, 'String', 0);
    errordlg('Input must be a number','Error');
end
T_hi = Theta_hi; 


% --- Executes during object creation, after setting all properties.
function Theta_hi_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_lo_Callback(hObject, eventdata, handles)
global T_lo
Theta_lo = str2double(get(hObject,'String'));   % returns contents of Theta_lo as a double
if isnan(Theta_lo)
    %set(hObject, 'String', 0);
    errordlg('Input must be a number','Error');
end
T_lo = Theta_lo; 
% --- Executes during object creation, after setting all properties.
function Theta_lo_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --------------------------------------------------------------------
function eegfile_open_Callback(hObject, eventdata, handles)
% This function is used to call the EEG file name.
working_dir=pwd;
current_dir='C:\SleepData\Datafiles';
cd(current_dir);
[filename, pathname] = uigetfile({'*.dat;*.Ncs;*.ncs;*.eeg; *.ascii',...
        'All data files (*.dat, *.Ncs, *.ncs, *.eeg, *.ascii)'}, 'Pick an EEG data file');
if isequal(filename,0) || isequal(pathname,0)
    uiwait(errordlg('You need to select a file. Please press the button again',...
        'ERROR','modal'));
    cd(working_dir);
else
    cd(working_dir);
    eegfile= fullfile(pathname, filename);
    set(handles.eegfile,'string',filename);
    set(handles.eegfile,'Tooltipstring',eegfile); 
end

% --------------------------------------------------------------------
function tstampsfile_open_Callback(hObject, eventdata, handles)
% This function is used to call the time stamp file name.
working_dir=pwd;
current_dir='C:\SleepData\Timestampfiles';
cd(current_dir);
[filename, pathname] = uigetfile('*.xls', 'Pick the timestamp file for these datafiles');
if isequal(filename,0) || isequal(pathname,0)
    uiwait(errordlg('You need to select a file. Please press the button again',...
        'ERROR','modal'));
    cd(working_dir);
else
    cd(working_dir);
    timestampfile= fullfile(pathname, filename);
    set(handles.tstampsfile,'string',filename);
    set(handles.tstampsfile,'Tooltipstring',timestampfile);
end



function EEG_cutoff_Callback(hObject, eventdata, handles)
global EEG_Fc
EEG_cutoff = str2double(get(hObject,'String'));   % returns contents of EEG_cutoff as a double
if isnan(EEG_cutoff)
    %set(hObject, 'String', 0);
    errordlg('Input must be a number','Error');
end
if EEG_cutoff > 124
    %set(hObject, 'String', 0);
    errordlg('Cutoff frequency must be < 125 Hz','Error');
end
EEG_Fc = EEG_cutoff;


% --- Executes during object creation, after setting all properties.
function EEG_cutoff_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in analyzePushButton.
function analyzePushButton_Callback(hObject, eventdata, handles)
global EEG_SAMPLES EEG_TIMESTAMPS EPOCHSIZE 
global T_lo T_hi T_Threshold
global EEG_Fc gapTime
global exactLow exactHi boundIndex
global LBounds UBounds BoundIndex Fs SampFreq2 sampfactor exactLowIndx exactHiIndx

% Get UI file names
filename2=get(handles.eegfile,'TooltipString');
filename3=get(handles.tstampsfile,'TooltipString');
working_dir=pwd;
current_dir='C:\SleepData\Results';
cd(current_dir);
[filename,pathname] = uiputfile('*ThetaTimeStamps.xls','Save Theta time stamp file as:');
cd(working_dir);

% Read in the timestampsfilebutton file.
[tbounds] = xlsread(filename3);
LBounds = tbounds(1:end,1);  UBounds = tbounds(1:end,2);
exactLowIndx = tbounds(1:end,3); exactHiIndx = tbounds(1:end,4);
thetaArray = [];
thetaTimeStamps = [];
msgid = 'signal:spectrum:obsoleteFunction'; %ID for the Spectrum warning.
warning('off', msgid); %Hide the Spectrum warning.
for boundIndex=1:length(UBounds)
    indexTheta = [];
%     waithandle= waitbar(0,'Loading the file ..... ');pause(0.2)
%     handles=guihandles(sleepscorer);
    exactLow = exactLowIndx(boundIndex);
    exactHi = exactHiIndx(boundIndex);

    %  ******    EEG FILE extraction   *********
    lowertimestamp=LBounds(boundIndex);
    uppertimestamp=UBounds(boundIndex);

%     waitbar(0.6,waithandle,' Converting EEG from Neuralynx CSC to Matlab data ...');
%     figure(waithandle),pause(0.2)
    [Timestamps,SF,Samples]=Nlx2MatCSC(filename2,[1 0 1 0 1],0,4,[lowertimestamp uppertimestamp]);
    samples=double(Samples(:)');
    clear Samples
    SampFreq2=SF(1);
    if boundIndex == 1
        DS = (1:1:10);
        DSampSF = SampFreq2./DS;
        indSampfactor = find(DSampSF >= 600);
        Fs = round(DSampSF(indSampfactor(end)));
        sampfactor = DS(indSampfactor(end));
         msgbox({['Orginal Sampling Rate:  ' num2str(SampFreq2) 'Hz'];...
            ['Down-Sampled Sampling Rate:  ' num2str(Fs) 'Hz']; ['Sampling Factor:  ' num2str(sampfactor) '']});
        binSize = Fs * EPOCHSIZE;
    end
%     waitbar(0.8,waithandle,'Filtering the EEG data ...'); 
%     figure(waithandle),pause(0.2),
    [EEG_TIMESTAMPS,EEG_SAMPLES] = generate_timestamps_from_Ncsfiles(Timestamps,samples, exactLow, exactHi);
    physInput = 2;  %Needed to select proper error box in HeaderADBit.
    ADBit2uV = HeaderADBit(filename2, physInput);    %Calls a function to extract the AD Bit Value.
    EEG_SAMPLES = EEG_SAMPLES * ADBit2uV;   %Convert EEG amplitude of signal from AD Bits to microvolts.
    %  Low pass filter for EEG signals
    [Blow,Alow]=ellip(7,1,60, EEG_Fc/(SampFreq2/2));  % Default setting implements low pass filter with 30hz cutoff
    filtered_samples=filter(Blow,Alow,EEG_SAMPLES);
    %  OPTIONAL highpass filter for EEG signals
   
    EEG_TIMESTAMPS = EEG_TIMESTAMPS(1:sampfactor:end);
    EEG_SAMPLES = filtered_samples(1:sampfactor:end);
    numberLoops = floor(length(EEG_SAMPLES)/binSize);
    clear physInput ADBit2uV
    clear filtered_samples samples
    % Calculate FFT(s)
    % This is calculating power in frequency domain
    windowsize = binSize;
    for i = 1:numberLoops
        startPoint = (i-1)*binSize + 1;
        endPoint = startPoint + binSize - 1;
        binDataEEG = EEG_SAMPLES(startPoint:endPoint);
        binTimeStamp = EEG_TIMESTAMPS(startPoint);
        
        [Pxx2,F2]=spectrum(binDataEEG,windowsize,0,ones(windowsize,1),Fs);
        % ******  [P,F] = SPECTRUM(X,NFFT,NOVERLAP,WINDOW,Fs)

        %For the EEG signal
        indexTheta=find(F2(1)+T_lo< F2 & F2 < F2(1)+T_hi); % Default delta band 5Hz to 9Hz
        thetaPower = sum(Pxx2(indexTheta))/windowsize *2;  
        thetaArray = [thetaArray;[binTimeStamp thetaPower]];
    end
end
indexArray = find(thetaArray(:,2) > T_Threshold);
thetaArray = thetaArray(indexArray,:);
lengthArray = size(thetaArray,1);
if isempty(thetaArray)
    errordlg('No theta under chosen settings','Error');
end
thetaTimeStamps = [thetaArray(1,1) thetaArray(1,1)];
for j = 2:lengthArray
    
    if thetaArray(j,1) < (thetaArray(j-1,1) + EPOCHSIZE + gapTime)
        thetaTimeStamps(end,2) = thetaArray(j,1);
    else
        thetaTimeStamps = [thetaTimeStamps; thetaArray(j,1) thetaArray(j,1)];
    end
end
save(filename,'thetaTimeStamps')

function epochSizeMenu_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function epochSizeMenu_Callback(hObject, eventdata, handles)
global EPOCHSIZE
epochVal = get(handles.epochSizeMenu,'Value');

switch epochVal
    case 1.0
        % User selected 1 sec epoch
        EPOCHSIZE = 1;        
    case 2.0
        % User selected 2 sec epoch
        EPOCHSIZE = 2;
    case 3.0
        % User selected 5 sec epoch
        EPOCHSIZE = 5;
    case 4.0
        % User selected 10 sec epoch
        EPOCHSIZE = 10;
    case 5.0
        % User selected 30 sec epoch
        EPOCHSIZE = 30;
end



function thetaGapTime_Callback(hObject, eventdata, handles)
global gapTime
t_gapTime = str2double(get(hObject,'String'));   % returns contents of t_gapTime as a double
if isnan(t_gapTime)
    %set(hObject, 'String', 0);
    errordlg('Input must be a number','Error');
end
if t_gapTime < 0
    %set(hObject, 'String', 0);
    errordlg('Input must be a positive number','Error');
end
gapTime = t_gapTime;

% --- Executes during object creation, after setting all properties.
function thetaGapTime_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function loadFilesMenu_Callback(hObject, eventdata, handles)
% hObject    handle to loadFilesMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
