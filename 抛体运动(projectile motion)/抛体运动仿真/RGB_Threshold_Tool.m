function varargout = RGB_Threshold_Tool(varargin)

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RGB_Threshold_Tool_OpeningFcn, ...
                   'gui_OutputFcn',  @RGB_Threshold_Tool_OutputFcn, ...
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


% --- Executes just before RGB_Threshold_Tool is made visible.
function RGB_Threshold_Tool_OpeningFcn(hObject, eventdata, handles, varargin)


RGB_Thresh = [0 255 0 255 0 255];
if ~isempty(varargin)
    frame = varargin{1};
else
    [fname,pname] = uigetfile('*.MOV','Select a video file','MultiSelect','off');
    vid = VideoReader(fullfile(pname,fname));
    
    frame = read(vid,1);
end

if length(varargin) > 1
    RGB_Thresh = varargin{2};
end

handles.hOrigImage = imshow(frame,'Parent',handles.axes1);
handles.hProcImage = imshow(frame,'Parent',handles.axes2);

[~,img] = createMask_RGB(frame,RGB_Thresh);
handles.hProcImage.CData = img;

handles.figure1.WindowButtonUpFcn = @unclickFcn;

axis(handles.axesRed,[-5 260 0 1])
axis(handles.axesGreen,[-5 260 0 1])
axis(handles.axesBlue,[-5 260 0 1])
hRedPatch = patch(handles.axesRed,[RGB_Thresh(1) RGB_Thresh(2) RGB_Thresh(2) RGB_Thresh(1)],[0 0 1 1],'c','FaceColor',[0.98, 0.922, 0.871],'EdgeColor','black');
hRedMin = line([RGB_Thresh(1) RGB_Thresh(1)],[0 1],'Parent',handles.axesRed,'LineWidth',2);
hRedMax = line([RGB_Thresh(2) RGB_Thresh(2)],[0 1],'Parent',handles.axesRed,'LineWidth',2);
hGreenPatch = patch(handles.axesGreen,[RGB_Thresh(3) RGB_Thresh(4) RGB_Thresh(4) RGB_Thresh(3)],[0 0 1 1],'c','FaceColor',[0.98, 0.922, 0.871],'EdgeColor','black');
hGreenMin = line([RGB_Thresh(3) RGB_Thresh(3)],[0 1],'Parent',handles.axesGreen,'LineWidth',2);
hGreenMax = line([RGB_Thresh(4) RGB_Thresh(4)],[0 1],'Parent',handles.axesGreen,'LineWidth',2);
hBluePatch = patch(handles.axesBlue,[RGB_Thresh(5) RGB_Thresh(6) RGB_Thresh(6) RGB_Thresh(5)],[0 0 1 1],'c','FaceColor',[0.98, 0.922, 0.871],'EdgeColor','black');
hBlueMin = line([RGB_Thresh(5) RGB_Thresh(5)],[0 1],'Parent',handles.axesBlue,'LineWidth',2);
hBlueMax = line([RGB_Thresh(6) RGB_Thresh(6)],[0 1],'Parent',handles.axesBlue,'LineWidth',2);
handles.axesRed.YTick = [];
handles.axesRed.XTick = 0:50:255;
handles.axesRed.Box = 'on';
handles.axesGreen.YTick = [];
handles.axesGreen.XTick = 0:50:255;
handles.axesGreen.Box = 'on';
handles.axesBlue.YTick = [];
handles.axesBlue.XTick = 0:50:255;
handles.axesBlue.Box = 'on';

handles.hRedPatch = hRedPatch;
handles.hRedMin = hRedMin;
handles.hRedMax = hRedMax;
handles.hGreenPatch = hGreenPatch;
handles.hGreenMin = hGreenMin;
handles.hGreenMax = hGreenMax;
handles.hBluePatch = hBluePatch;
handles.hBlueMin = hBlueMin;
handles.hBlueMax = hBlueMax;
handles.minRed = RGB_Thresh(1);
handles.maxRed = RGB_Thresh(2);
handles.minGreen = RGB_Thresh(3);
handles.maxGreen = RGB_Thresh(4);
handles.minBlue = RGB_Thresh(5);
handles.maxBlue = RGB_Thresh(6);

handles.editRedMin.String = handles.minRed;
handles.editRedMax.String = handles.maxRed;
handles.editGreenMin.String = handles.minGreen;
handles.editGreenMax.String = handles.maxGreen;
handles.editBlueMin.String = handles.minBlue;
handles.editBlueMax.String = handles.maxBlue;

hRedMin.ButtonDownFcn = {@clickFcn, handles};
hRedMax.ButtonDownFcn = {@clickFcn, handles};
hGreenMin.ButtonDownFcn = {@clickFcn, handles};
hGreenMax.ButtonDownFcn = {@clickFcn, handles};
hBlueMin.ButtonDownFcn = {@clickFcn, handles};
hBlueMax.ButtonDownFcn = {@clickFcn, handles};

handles.text2.String = msg('RGB_text2');
handles.text3.String = msg('RGB_text3');
handles.text4.String = msg('RGB_text4');
handles.text5.String = msg('RGB_text5');
handles.text6.String = msg('RGB_text6');
handles.text7.String = msg('RGB_text7');
handles.pushbutton1.String = msg('RGB_pushbutton1');

handles.text2.FontName = msg('FontName');
handles.text3.FontName = msg('FontName');
handles.text4.FontName = msg('FontName');
handles.text5.FontName = msg('FontName');
handles.text6.FontName = msg('FontName');
handles.text7.FontName = msg('FontName');
handles.pushbutton1.FontName = msg('FontName');

% Choose default command line output for RGB_Threshold_Tool
handles.output = [];

% Update handles structure
guidata(hObject, handles);

movegui(hObject,'center')

handles.figure1.WindowStyle = 'modal';
% UIWAIT makes RGB_Threshold_Tool wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RGB_Threshold_Tool_OutputFcn(hObject, eventdata, handles) 


% Get default command line output from handles structure
if ~isempty(handles)
    varargout{1} = handles.output;
    close(hObject)
else
    varargout{1} = [];
end


function editRedMin_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function editRedMin_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editRedMax_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function editRedMax_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editGreenMin_Callback(hObject, eventdata, handles)

function editGreenMin_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editGreenMax_Callback(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editBlueMin_Callback(hObject, eventdata, handles)

function editBlueMin_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editBlueMax_Callback(hObject, eventdata, handles)


if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function clickFcn(obj, edata, handles)
switch obj
    case handles.hRedMin
        handles.figure1.WindowButtonMotionFcn = {@dragFcn,handles.axesRed,obj,handles.editRedMin,handles.hRedPatch,handles.hRedMax,handles};
    case handles.hRedMax
        handles.figure1.WindowButtonMotionFcn = {@dragFcn,handles.axesRed,obj,handles.editRedMax,handles.hRedPatch,handles.hRedMin,handles};
    case handles.hGreenMin
        handles.figure1.WindowButtonMotionFcn = {@dragFcn,handles.axesGreen,obj,handles.editGreenMin,handles.hGreenPatch,handles.hGreenMax,handles};
    case handles.hGreenMax
        handles.figure1.WindowButtonMotionFcn = {@dragFcn,handles.axesGreen,obj,handles.editGreenMax,handles.hGreenPatch,handles.hGreenMin,handles};
    case handles.hBlueMin
        handles.figure1.WindowButtonMotionFcn = {@dragFcn,handles.axesBlue,obj,handles.editBlueMin,handles.hBluePatch,handles.hBlueMax,handles};
    case handles.hBlueMax
        handles.figure1.WindowButtonMotionFcn = {@dragFcn,handles.axesBlue,obj,handles.editBlueMax,handles.hBluePatch,handles.hBlueMin,handles};
end


function dragFcn(obj,edata,hAxes,hLine,hEdit,hPatch,hLine2,handles)

pt = hAxes.CurrentPoint;
val = max(0,min(255,round(pt(1,1))));
hLine.XData = [val val];
hEdit.String = val;
hPatch.XData = [val hLine2.XData val];

rMin = str2double(handles.editRedMin.String);
rMax = str2double(handles.editRedMax.String);
gMin = str2double(handles.editGreenMin.String);
gMax = str2double(handles.editGreenMax.String);
bMin = str2double(handles.editBlueMin.String);
bMax = str2double(handles.editBlueMax.String);


[~,img] = createMask_RGB(handles.hOrigImage.CData,[rMin rMax gMin gMax bMin bMax]);
handles.hProcImage.CData = img;


function unclickFcn(obj, edata)

obj.WindowButtonMotionFcn = '';


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)

rMin = round(str2double(handles.editRedMin.String));
rMax = round(str2double(handles.editRedMax.String));
gMin = round(str2double(handles.editGreenMin.String));
gMax = round(str2double(handles.editGreenMax.String));
bMin = round(str2double(handles.editBlueMin.String));
bMax = round(str2double(handles.editBlueMax.String));

handles.output = [rMin rMax gMin gMax bMin bMax];

guidata(hObject,handles)

uiresume(gcf)
