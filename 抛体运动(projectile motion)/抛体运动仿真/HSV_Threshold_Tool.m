function varargout = HSV_Threshold_Tool(varargin)
% HSV_THRESHOLD_TOOL MATLAB code for HSV_Threshold_Tool.fig
%      HSV_THRESHOLD_TOOL, by itself, creates a new HSV_THRESHOLD_TOOL or raises the existing
%      singleton*.
%
%      H = HSV_THRESHOLD_TOOL returns the handle to a new HSV_THRESHOLD_TOOL or the handle to
%      the existing singleton*.
%
%      HSV_THRESHOLD_TOOL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HSV_THRESHOLD_TOOL.M with the given input arguments.
%
%      HSV_THRESHOLD_TOOL('Property','Value',...) creates a new HSV_THRESHOLD_TOOL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before HSV_Threshold_Tool_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to HSV_Threshold_Tool_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help HSV_Threshold_Tool

% Copyright 2018-2019 The MathWorks, Inc.

% Last Modified by GUIDE v2.5 09-Oct-2018 00:14:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @HSV_Threshold_Tool_OpeningFcn, ...
                   'gui_OutputFcn',  @HSV_Threshold_Tool_OutputFcn, ...
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


% --- Executes just before HSV_Threshold_Tool is made visible.
function HSV_Threshold_Tool_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to HSV_Threshold_Tool (see VARARGIN)

HSV_Thresh = [0 1];
if ~isempty(varargin)
    frame = varargin{1};
else
    [fname,pname] = uigetfile('*.MOV','Select a video file','MultiSelect','off');
    vid = VideoReader(fullfile(pname,fname));
    
    frame = read(vid,1);
end

if length(varargin) > 1
    HSV_Thresh = varargin{2};
end

handles.hOrigImage = imshow(frame,'Parent',handles.axes1);
handles.hProcImage = imshow(frame,'Parent',handles.axes2);

handles.figure1.UserData = HSV_Thresh;

HSVpie(handles.hProcImage,handles.axes6,HSV_Thresh);

handles.text5.String = msg('HSV_text5');
handles.text6.String = msg('HSV_text6');
handles.text7.String = msg('HSV_text7');
handles.pushbutton1.String = msg('HSV_pushbutton1');

handles.text5.FontName = msg('FontName');
handles.text6.FontName = msg('FontName');
handles.text7.FontName = msg('FontName');
handles.pushbutton1.FontName = msg('FontName');

% Choose default command line output for HSV_Threshold_Tool
handles.output = [];

% Update handles structure
guidata(hObject, handles);

movegui(hObject,'center')

handles.figure1.WindowStyle = 'modal';
% UIWAIT makes HSV_Threshold_Tool wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = HSV_Threshold_Tool_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
if ~isempty(handles)
    varargout{1} = handles.output;
    close(hObject)
else
    varargout{1} = [];
end

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.output = handles.figure1.UserData;

guidata(hObject,handles)

uiresume(gcf)
