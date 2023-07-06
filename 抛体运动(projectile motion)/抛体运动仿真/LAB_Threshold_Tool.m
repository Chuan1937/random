function varargout = LAB_Threshold_Tool(varargin)

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LAB_Threshold_Tool_OpeningFcn, ...
                   'gui_OutputFcn',  @LAB_Threshold_Tool_OutputFcn, ...
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


% --- Executes just before LAB_Threshold_Tool is made visible.
function LAB_Threshold_Tool_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LAB_Threshold_Tool (see VARARGIN)

LAB_Thresh = [-50 50];
if ~isempty(varargin)
    frame = varargin{1};
else
    [fname,pname] = uigetfile('*.MOV','Select a video file','MultiSelect','off');
    vid = VideoReader(fullfile(pname,fname));
    
    frame = read(vid,1);
end

if length(varargin) > 1
    LAB_Thresh = varargin{2};
end

handles.hOrigImage = imshow(frame,'Parent',handles.axes1);
handles.hProcImage = imshow(frame,'Parent',handles.axes2);

handles.frame = frame;
handles.frame_lab = rgb2lab(frame);

[~,img] = createMask_LAB(frame,rgb2lab(frame),LAB_Thresh);
handles.hProcImage.CData = img;

handles.figure1.WindowButtonUpFcn = @unclickFcn;

axis(handles.axesA,[-55 55 0 1])
hAPatch = patch(handles.axesA,[LAB_Thresh(1) LAB_Thresh(2) LAB_Thresh(2) LAB_Thresh(1)],[0 0 1 1],'c','FaceColor',[0.98, 0.922, 0.871],'EdgeColor','black');
hAMin = line([LAB_Thresh(1) LAB_Thresh(1)],[0 1],'Parent',handles.axesA,'LineWidth',2);
hAMax = line([LAB_Thresh(2) LAB_Thresh(2)],[0 1],'Parent',handles.axesA,'LineWidth',2);
handles.axesA.YTick = [];
handles.axesA.XTick = 0:50:255;
handles.axesA.Box = 'on';

handles.hAPatch = hAPatch;
handles.hAMin = hAMin;
handles.hAMax = hAMax;
handles.minA = LAB_Thresh(1);
handles.maxA = LAB_Thresh(2);

handles.editAMin.String = handles.minA;
handles.editAMax.String = handles.maxA;

hAMin.ButtonDownFcn = {@clickFcn, handles};
hAMax.ButtonDownFcn = {@clickFcn, handles};

handles.text5.String = msg('LAB_text5');
handles.text6.String = msg('LAB_text6');
handles.text8.String = msg('LAB_text8');
handles.pushbutton1.String = msg('LAB_pushbutton1');

handles.text5.FontName = msg('FontName');
handles.text6.FontName = msg('FontName');
handles.text8.FontName = msg('FontName');
handles.pushbutton1.FontName = msg('FontName');

% Choose default command line output for LAB_Threshold_Tool
handles.output = [];

% Update handles structure
guidata(hObject, handles);

movegui(hObject,'center')

handles.figure1.WindowStyle = 'modal';
% UIWAIT makes LAB_Threshold_Tool wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LAB_Threshold_Tool_OutputFcn(hObject, eventdata, handles) 
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


function editAMin_Callback(hObject, eventdata, handles)
% hObject    handle to editAMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAMin as text
%        str2double(get(hObject,'String')) returns contents of editAMin as a double


% --- Executes during object creation, after setting all properties.
function editAMin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAMax_Callback(hObject, eventdata, handles)
% hObject    handle to editAMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAMax as text
%        str2double(get(hObject,'String')) returns contents of editAMax as a double


% --- Executes during object creation, after setting all properties.
function editAMax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function clickFcn(obj, edata, handles)
switch obj
    case handles.hAMin
        handles.figure1.WindowButtonMotionFcn = {@dragFcn,handles.axesA,obj,handles.editAMin,handles};
    case handles.hAMax
        handles.figure1.WindowButtonMotionFcn = {@dragFcn,handles.axesA,obj,handles.editAMax,handles};
end


function dragFcn(obj,edata,hAxes,hLine,hEdit,handles)

pt = hAxes.CurrentPoint;
val = max(-50,min(50,round(pt(1,1))));
hLine.XData = [val val];
hEdit.String = val;
handles.hAPatch.XData = [handles.hAMin.XData(1) handles.hAMax.XData(1) handles.hAMax.XData(1) handles.hAMin.XData(1)];

aMin = str2double(handles.editAMin.String);
aMax = str2double(handles.editAMax.String);

[~,img] = createMask_LAB(handles.frame,handles.frame_lab,[aMin aMax]);
handles.hProcImage.CData = img;


function unclickFcn(obj, edata)

obj.WindowButtonMotionFcn = '';


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

aMin = round(str2double(handles.editAMin.String));
aMax = round(str2double(handles.editAMax.String));


handles.output = [aMin aMax];

guidata(hObject,handles)

uiresume(gcf)
