function varargout = trim_video(varargin)
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @trim_video_OpeningFcn, ...
    'gui_OutputFcn',  @trim_video_OutputFcn, ...
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


% --- Executes just before trim_video is made visible.
function trim_video_OpeningFcn(hObject, eventdata, handles, varargin)

[fname,pname] = uigetfile('*.mov;*.avi;*.mp4;*.mts',msg('SelectVideo'));
if ~isnumeric(fname)
    filename = fullfile(pname,fname);
    handles.filename = filename;
    
    warnState = warning('off','MATLAB:subscripting:noSubscriptsSpecified');
    handles.vidObj = VideoReader(filename);
    warning(warnState)
    
    if handles.vidObj.FrameRate > 30
        interval = ceil(handles.vidObj.FrameRate/30);
    else
        interval = 1;
    end
    handles.frame_rate = handles.vidObj.FrameRate/interval;
    
    numFrames = handles.vidObj.Duration*handles.vidObj.FrameRate;
    readFrames = 1:interval:numFrames;
    tmpFrame = read(handles.vidObj,1);
    tmpFrames = zeros([size(tmpFrame) length(readFrames)],'uint8');
    tmpFrames(:,:,:,1) = tmpFrame;
    id = 2;
    hWait = waitbar(0,msg('LoadingVideo'));
    for ii = 2:length(readFrames)
        waitbar(ii/length(readFrames),hWait)
        try
            f = read(handles.vidObj,readFrames(ii));
        catch
            break
        end
        tmpFrames(:,:,:,id) = f;
        id = id+1;
    end
    delete(hWait)
    
    handles.frames_rgb = tmpFrames;
    
    handles.nof = size(handles.frames_rgb,4);
    
    handles.imgf = handles.frames_rgb(:,:,:,1);
    handles.imgl = handles.frames_rgb(:,:,:,end);
    
    handles.hFirstIm = imshow(handles.imgf,'Parent',handles.axes1);
    handles.hLastIm = imshow(handles.imgl,'Parent',handles.axes2);
    
    set(handles.edit1,'String',num2str(1))
    set(handles.edit2,'String',num2str(handles.nof))
    
    set(handles.axes3,'XLim',[1 handles.nof])
    xlimdata = xlim(handles.axes3);
    %     ylimdata = ylim(handles.axes3);
    
    handles.lhf = plot(handles.axes3,[1 1],[0 1],'LineWidth',3);
    hold(handles.axes3,'on')
    handles.lhl = plot(handles.axes3,handles.nof.*[1 1],[0 1],'LineWidth',3);
    handles.hp = patch(handles.axes3,[(xlimdata(1)),(xlimdata(1)),(xlimdata(2)),(xlimdata(2))],[0,1,1,0],'r','facealpha',0.5,'edgecolor','none','facecolor',[0.98, 0.922, 0.871]);
    hold(handles.axes3,'off')
    
    set(handles.axes3,'XLim',[1 handles.nof])
    set(handles.axes3,'YTick',[])
    
    set(handles.figure1,'WindowButtonMotionFcn',{@movefun, handles})
    set(handles.figure1,'WindowButtonDownFcn',{@downfun, handles})
    set(handles.figure1,'WindowButtonUpFcn',{@upfun, handles})
    
    handles.text2.String = msg('trim_video_text2');
    handles.text3.String = msg('trim_video_text3');
    handles.text4.String = msg('trim_video_text4');
    handles.pushbutton2.String = msg('trim_video_pushbutton2');

    handles.text2.FontName = msg('FontName');
    handles.text3.FontName = msg('FontName');
    handles.text4.FontName = msg('FontName');
    handles.pushbutton2.FontName = msg('FontName');

    % guidata(hObject, handles);
    
    % Choose default command line output for trim_video
    handles.output = hObject;
    
    % Update handles structure
    guidata(hObject, handles);
    
    movegui(hObject,'center')
    
    % UIWAIT makes trim_video wait for user response (see UIRESUME)
    uiwait(handles.figure1);
else
    
    handles.output = [];
    guidata(hObject, handles);
    %close(hObject)
end


% --- Outputs from this function are returned to the command line.
function varargout = trim_video_OutputFcn(hObject, eventdata, handles)

% Get default command line output from handles structure
if ~isempty(handles)
    varargout{1} = handles.output;
    close(hObject)
else
    varargout{1} = [];
end

function edit1_Callback(hObject, eventdata, handles)

  
% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)


indf = str2double(get(handles.edit1,'String'));
indl = str2double(get(handles.edit2,'String'));

tmpFrames = handles.frames_rgb(:,:,:,indf:indl);

if size(tmpFrames,1) > 800
    hWait = waitbar(1,'切割中...');
    tmpFrames = imresize(tmpFrames,[800 nan]);
    delete(hWait)
end

handles.output = {tmpFrames, handles.frame_rate};

guidata(hObject,handles)

uiresume(handles.figure1)
%close(gcf)

function movefun(obj,event, handles)
% 获取当前鼠标位置
pos_move = get(gca,'currentpoint');
mdflag = get(handles.figure1,'userdata');
% 只能在Axes范围内移动
if min(xlim) <= pos_move(1) && max(xlim) >= pos_move(1)
    h1x = get(handles.lhf,'XData');
    h2x = get(handles.lhl,'XData');
    
    % 鼠标单击和拖动期间的操作
    if mdflag == 1
        dist1 = abs(pos_move(1)-h1x(1));
        dist2 = abs(pos_move(1)-h2x(1));
        
        % 如果鼠标选择靠近线，将线移动到鼠标位置
        if h1x(1) > h2x(1)
            set(handles.lhf,'XData',h2x)
            set(handles.lhl,'XData',h1x)
        elseif dist1 <= dist2 && dist1 < max(xlim)./20
            set(handles.lhf,'XData',[pos_move(1),pos_move(1)])
            set(handles.hp,'XData',sort([pos_move(1),pos_move(1), h2x]))
            
            handles.hFirstIm.CData = handles.frames_rgb(:,:,:,round(pos_move(1)));
            handles.edit1.String = round(pos_move(1));
        elseif dist1 > dist2 && dist2 < max(xlim)./20
            set(handles.lhl,'Xdata',[pos_move(1),pos_move(1)])
            set(handles.hp,'XData',sort([pos_move(1),pos_move(1), h1x]))
            
            handles.hLastIm.CData = handles.frames_rgb(:,:,:,round(pos_move(1)));
            handles.edit2.String = round(pos_move(1));
        end
    end
end

function downfun(obj,event,handles)
% 鼠标点击标志
mdflag = 1;
set(handles.figure1,'userdata',mdflag)


function upfun(obj,event,handles)
mdflag = 0;
set(handles.figure1,'userdata',mdflag)

h1x = round(get(handles.lhf,'XData'));
h2x = round(get(handles.lhl,'XData'));

set(handles.edit1,'String',num2str(h1x(1)))
set(handles.edit2,'String',num2str(h2x(1)))

handles.hFirstIm.CData = handles.frames_rgb(:,:,:,h1x(1));
handles.hLastIm.CData = handles.frames_rgb(:,:,:,h2x(1));
