function varargout = ParabolaApp(varargin)

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ParabolaApp_OpeningFcn, ...
                   'gui_OutputFcn',  @ParabolaApp_OutputFcn, ...
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


% --- Executes just before ParabolaApp is made visible.
function ParabolaApp_OpeningFcn(hObject, eventdata, handles, varargin)


if ~ispref('ParabolaApp','lang')
    btn = questdlg('Choose language','Language','English','中文','中文');
    if isempty(btn)
        lang = 'cn';
    else
        switch btn
            case 'English'
                lang = 'en';
            case '中文'
                lang = 'cn';
        end
    end
    setpref('ParabolaApp','lang',lang)
end
   
[langCode, langName] = languages;
langPref = getpref('ParabolaApp','lang');
for id = 1:length(langCode)
    handles.LanguageItems(id) = uimenu(handles.Language_Menu,'Text',langName{id},'UserData',langCode{id});
    if strcmpi(langCode{id},langPref)
        handles.LanguageItems(id).Checked = 'on';
    else
        handles.LanguageItems(id).Checked = 'off';
    end
end

set(handles.LanguageItems,'MenuSelectedFcn',{@LanguageItemsCallback,handles})

updateLanguage(handles)

handles.RGB_Thresh = [0 255 0 255 0 255];
handles.HSV_Thresh = [0 1];
handles.LAB_Thresh = [-50 50];

handles.isProcessed = false;

imshow('x_equals.png','Parent',handles.axes15)
imshow('y_equals.png','Parent',handles.axes17)

% Choose default command line output for ParabolaApp
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

movegui(hObject,'north')

% UIWAIT makes ParabolaApp wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ParabolaApp_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;



% --- Executes on button press in pushbutton2.
function track_object(hObject,handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% imtool close
%% 各种System Objects生成 %%%%%
% 为视频加载创建对象


handles.blobAnalyser = vision.BlobAnalysis( 'AreaOutputPort',false, ...
    'CentroidOutputPort', true,  'BoundingBoxOutputPort', true, ...
    'MinimumBlobArea', 20, 'ExcludeBorderBlobs',true);

% 初始化轨道（每个轨道存储每个移动物体的信息）
tracks = struct(...
    'id',           {}, ...       % ID番号
    'bbox',         {}, ...       %当前帧中的边界框（用于显示）
    'kalmanFilter', {}, ...       %用于此对象跟踪的卡尔曼滤波器
    'age',               {}, ...  %自首次检测以来的帧数
    'totalVisibleCount', {}, ...      %检测到的帧总数 => 显示是否超过阈值
    'consecutiveInvisibleCount', {}); %连续未检测到的帧数=>如果超过阈值则删除该轨道

nextId = 1; % ID of the next track
centroidLog = [];

%%
% minVal = handles.minVal2;
% maxVal = handles.maxVal2;
%% 主循环 %%%%%
hWait = waitbar(1,msg('TrackingObject'));
switch handles.colorType
    case 'rgb'
        use_frames = handles.vidData;
    case 'hsv'
        [a,b,c,d] = size(handles.vidData);
        tmp = mat2cell(handles.vidData,a,b,c,ones(1,d));
        xx = cat(1,tmp{:});
        xxx = mat2cell(rgb2hsv(xx),a*ones(1,d),b,c);
        use_frames = cat(4,xxx{:});

        %use_frames = rgb2hsv(handles.vidData);
    case 'lab'
        use_frames = rgb2lab(handles.vidData);
end
delete(hWait)
for iFrame = 1:size(handles.vidData,4)
%while ( isDone(handles.vidObj)==false)
    frame_rgb = handles.vidData(:,:,:,iFrame);     % 1フレーム読み込み
        %% 【检测画面中所有运动物体】 %%%%%
    % Detect objects in the frame: Detect moving objects (areas): mask为1前景，0为背景
    switch handles.colorType
        case 'rgb'
            mask = createMask_RGB(use_frames(:,:,:,iFrame),handles.RGB_Thresh);
        case 'hsv'
            mask = createMask_HSV(frame_rgb,use_frames(:,:,:,iFrame),handles.HSV_Thresh(1),handles.HSV_Thresh(2));
        case 'lab'
            mask = createMask_LAB(frame_rgb,use_frames(:,:,:,iFrame),handles.LAB_Thresh);
    end
    %mask = createMask3(handles.frames_lab(:,:,:,iFrame),minVal,maxVal);
     mask = bwareafilt(mask,1);
    % 中心点/边界框检测
    [centroids, bboxes] = step(handles.blobAnalyser, mask);
    
    %% [根据前一帧预测当前帧中的位置] %%%%%
    % 使用卡尔曼滤波器对到前一帧检测到的每个对象（轨道）的位置预测
    for i = 1:length(tracks)
        bbox = tracks(i).bbox;    % 前一帧中的边界框
        % 预测当前帧中的位置
        predictedPosition = int32(predict(tracks(i).kalmanFilter));
        % 将边界框中心调整到预测的质心位置
        tracks(i).bbox = [predictedPosition - bbox(3:4)/2, bbox(3:4)];
        
    end
    
    %% 将检测到的对象与已检测到的对象（轨迹）相关联 %%%%%
    % 成本的计算：预测已经检测到的物体的位置和每个检测到的物体的距离
    cost = zeros(length(tracks), size(centroids, 1));         % 轨迹数 x 检测数
    for i = 1:length(tracks)
        cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
    end
    
    % 将检测到的对象分配给每个轨道（匈牙利算法：最小化总成本）
    % assignment:代入一组轨道号和检测号
    costOfNonAssignment = 87;     %如果它很小，则不会将更多轨道分配给现有轨道 => 将生成许多新轨道
    [assignments, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(cost, costOfNonAssignment);
    
    %% 找到对应对象的轨迹信息更新%%%%%
    for i = 1:size(assignments, 1)
        trackIdx = assignments(i, 1);           
        detectionIdx = assignments(i, 2);       
        centroid = centroids(detectionIdx, :);  
        bbox = bboxes(detectionIdx, :);         
        
        correct(tracks(trackIdx).kalmanFilter, centroid);
        
       
        tracks(trackIdx).bbox = bbox;
        tracks(trackIdx).age = tracks(trackIdx).age + 1;
        tracks(trackIdx).totalVisibleCount = ...
            tracks(trackIdx).totalVisibleCount + 1;
        tracks(trackIdx).consecutiveInvisibleCount = 0;
    end
   
    if isempty(assignments)
        centroidLog = centroids;
    else
        [~,idx] = sort(assignments(:,1));
        centroidLog = [centroidLog centroids(assignments(idx,2),:)];
    end
    
    %% 未找到对应对象的轨道信息更新%%%%%
    
    for i = 1:length(unassignedTracks)
        ind = unassignedTracks(i);
        tracks(ind).age = tracks(ind).age + 1;
        tracks(ind).consecutiveInvisibleCount = ...
            tracks(ind).consecutiveInvisibleCount + 1;
    end
    
    
    %% 擦除丢失的曲目%%%%%
  
    if ~isempty(tracks)
        
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;     
        
        % find the indices of 'lost' tracks
        lostInds = (ages < 8 & visibility < 0.6) | ...
            ([tracks(:).consecutiveInvisibleCount] >= 100);   
        
        tracks = tracks(~lostInds);   
    end
    
    %% 为新发现的对象生成新轨迹%%%%%
    
    centroids1 = centroids(unassignedDetections, :);          
    bboxes = bboxes(unassignedDetections, :);                
    
    for i = 1:size(centroids1, 1)
        
        centroid = centroids1(i,:);
        bbox = bboxes(i, :);
        
       
        kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
            centroid, [200, 50], [100, 25], 100);
        
        
        newTrack = struct(...
            'id', nextId, ...
            'bbox', bbox, ...
            'kalmanFilter', kalmanFilter, ...
            'age', 1, ...
            'totalVisibleCount', 1, ...
            'consecutiveInvisibleCount', 0);
        
        
        tracks(end + 1) = newTrack;
        
        
        nextId = nextId + 1;
    end
    
    
    %% 结果表示 %%%%%
    frame_rgb = im2uint8(frame_rgb);
    mask = uint8(repmat(mask, [1, 1, 3])) .* 255;  
    
    if ~isempty(tracks)
        
        reliableTrackInds = ...
            [tracks(:).totalVisibleCount] < 8;         
        reliableTracks = tracks(reliableTrackInds);
        
        % display the objects. If an object has not been detected
        % in this frame, display its predicted bounding box.
        if ~isempty(reliableTracks)
            
            bboxes = cat(1, reliableTracks.bbox);
            
         
            ids = int32([reliableTracks(:).id]);
            
            
            labels = cellstr(int2str(ids'));
            predictedTrackInds = ...
                [reliableTracks(:).consecutiveInvisibleCount] > 0;
            isPredicted = cell(size(labels));
            isPredicted(predictedTrackInds) = {' predicted'};
            labels = strcat(labels, isPredicted);
            
            
            frame_rgb = insertObjectAnnotation(frame_rgb, 'rectangle', ...
                bboxes, labels);
            
         
            mask = insertObjectAnnotation(mask, 'rectangle', ...
                bboxes, labels);
            
            mask = insertMarker(mask, centroids, 'plus');     
            mask = insertMarker(mask, centroids1, 'plus', 'Color','red'); 
        end
    end
    
  
    if size(centroidLog,2) >= 30
        frame_rgb = insertShape(frame_rgb,'line',centroidLog,'LineWidth',5);
    end
    
    handles.procVidData(:,:,:,iFrame) = frame_rgb;
    
  
    showFrameOnAxis(handles.axes1, frame_rgb) % myzk

end

handles.isProcessed = true;

x = centroidLog(:,1:2:end)';
y = 480-centroidLog(:,2:2:end)';

if isempty(x)
    uiwait(msgbox(msg('CouldNotDetect')))
    return
end

x = x - x(1);
y = y - y(1);

% plot(x,y)
plot(handles.axes6,x,y,'o');
grid(handles.axes6,'on')
title(handles.axes6,msg('TrajectoryLabel')); xlabel(handles.axes6,msg('HorizontalPixel')); ylabel(handles.axes6,msg('VerticalPixel'));

% plot(t,x)
%S = handles.vidObj.info;
t = ((0:(size(centroidLog,2)/2-1))/handles.FrameRate)';
plot(handles.axes2,t,x,'o');
grid(handles.axes2,'on')
ylabel(handles.axes2,msg('HorizontalPixel')); 
xticklabels(handles.axes2,[])

% plot(t,y)
plot(handles.axes3,t,y,'o');
xlabel(handles.axes3,msg('TimeLabel')); ylabel(handles.axes3,msg('VerticalPixel'));
grid(handles.axes3,'on')

handles.t_all = t;
handles.x_all = x;
handles.y_all = y;
guidata(hObject,handles)

assignin('base','t',t)
assignin('base','x',x)
assignin('base','y',y)

handles.UnitConversion.Enable = 'on';
handles.LengthMeasurement.Enable = 'on';
handles.DataCursorMode.Enable = 'on';
%handles.ShowTheoretical.Enable = 'on';


function edit_vx0_Callback(hObject, eventdata, handles)

function edit_vx0_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_vy0_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit_vy0_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ShowTheoretical.
function ShowTheoretical_Callback(hObject, eventdata, handles)


pixel_dist = str2double(get(handles.edit_length_pixel,'String'));
actual_dist = str2double(get(handles.edit_length_cm,'String'));

if isnan(pixel_dist) || isnan(actual_dist) || pixel_dist == 0 || actual_dist == 0
    uiwait(msgbox(msg('ConvertFromPixelToCM'),'modal'))
    return
end 

try
    x_t_fcn_m = str2func(['@(t)' get(handles.edit_vx0,'String')]);
    y_t_fcn_m = str2func(['@(t)' get(handles.edit_vy0,'String')]);
    
 
    x_t_fcn_cm = @(t) 100*x_t_fcn_m(t);
    y_t_fcn_cm = @(t) 100*y_t_fcn_m(t);
    
    
    tmp = x_t_fcn_cm(1);
    tmp = y_t_fcn_cm(1);
catch
    uiwait(errordlg(msg('WrongFormulaEnterAgain'),'Error','modal'))
    return
end

x_vec = handles.x_all;
y_vec = handles.y_all;

time_vec = handles.t_all;

x_vec_actual = x_vec ./pixel_dist .* actual_dist;
y_vec_actual = y_vec ./pixel_dist .* actual_dist;

cla(handles.axes6)

% hLines = plot(handles.axes6,x_vec_actual,y_vec_actual,'o',x_ideal,y_ideal,'-');
hLines = plot(handles.axes6,x_vec_actual,y_vec_actual,'o');
hold(handles.axes6,'on')
warnState = warning('off','MATLAB:fplot:NotVectorized');
hLines(2) = fplot(handles.axes6,x_t_fcn_cm,y_t_fcn_cm,[0 time_vec(end)]);
warning(warnState)
hLines(2).LineWidth = 2;
hold(handles.axes6,'off')
legend(handles.axes6,msg('LegendMeasuredData'),msg('LegendTheoretical'),'Location','Northeast')

guidata(hObject, handles)

f = figure('Units','pixels','Position',[100 100 900 400],...
    'Toolbar','none','Menu','none','Name',msg('LetsThinkName'),'NumberTitle','off',...
    'Visible','off','WindowStyle','modal');
uicontrol('Units','normalized','Position',[0.1 0.1 0.8 0.8],...
    'Style','text','FontName',msg('FontName'),'String',msg('LetsThink'),...
    'FontSize',20,'FontWeight','bold','ForegroundColor','r','HorizontalAlignment','left')
uicontrol('Units','pixels','Position',[750 5 140 30],...
    'Style','pushbutton','String','OK','FontName',msg('FontName'),...
    'FontSize',16,'FontWeight','bold','Callback',@(o,e) delete(f))
movegui(f,'north')
f.Visible = 'on';
uiwait(f)


function edit_length_cm_Callback(hObject, eventdata, handles)
% Actual Distance
% hObject    handle to edit_length_cm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_length_cm as text
%        str2double(get(hObject,'String')) returns contents of edit_length_cm as a double


% --- Executes during object creation, after setting all properties.
function edit_length_cm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_length_cm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_length_pixel_Callback(hObject, eventdata, handles)
% Pixel length
% hObject    handle to edit_length_pixel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_length_pixel as text
%        str2double(get(hObject,'String')) returns contents of edit_length_pixel as a double


% --- Executes during object creation, after setting all properties.
function edit_length_pixel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_length_pixel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in UnitConversion.
function UnitConversion_Callback(hObject, eventdata, handles)
% hObject    handle to UnitConversion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of UnitConversion
pixel_dist = str2double(get(handles.edit_length_pixel,'String'));
actual_dist = str2double(get(handles.edit_length_cm,'String'));

if isnan(pixel_dist) || isnan(actual_dist) || pixel_dist == 0 || actual_dist == 0
    uiwait(msgbox(msg('ConvertFromPixelToCM'),'modal'))
    return
end

% (t,x) グラフ更新
lh2 = get(handles.axes2,'Children');
tx_vec = handles.x_all';
tx_vec_actual = tx_vec./pixel_dist .* actual_dist;
set(lh2,'YData',tx_vec_actual)
set(handles.axes2,'YLim',[round(min(tx_vec_actual)),ceil(max(tx_vec_actual))])
ylabel(handles.axes2,msg('HorizontalCM'));

lh3 = get(handles.axes3,'Children');
ty_vec = handles.y_all';
ty_vec_actual = ty_vec./pixel_dist .* actual_dist;

set(lh3,'YData',ty_vec_actual)
set(handles.axes3,'YLim',[round(min(ty_vec_actual)),ceil(max(ty_vec_actual))])
ylabel(handles.axes3,msg('VerticalCM'));

lh6 = get(handles.axes6,'Children');
x_vec = handles.x_all'; % pixel
y_vec = handles.y_all'; % pixel

x_vec_actual = x_vec ./pixel_dist .* actual_dist;
y_vec_actual = y_vec ./pixel_dist .* actual_dist;
% set(lh4,'XData',x_vec_actual,'YData',y_vec_actual)
% set(handles.axes4,'XLim',[round(min(x_vec_actual)),ceil(max(x_vec_actual))])
% set(handles.axes4,'YLim',[round(min(y_vec_actual)),ceil(max(y_vec_actual))])
% xlabel(handles.axes4,'X軸位置(cm)');
% ylabel(handles.axes4,'Y軸位置(cm)');

plot(handles.axes6,x_vec_actual-x_vec_actual(1),y_vec_actual-y_vec_actual(1),'o')
xlabel(handles.axes6,msg('HorizontalCM'));
ylabel(handles.axes6,msg('VerticalCM'));

handles.ShowTheoretical.Enable = 'on';

% --- Executes on button press in LengthMeasurement.
function LengthMeasurement_Callback(hObject, eventdata, handles)
% hObject    handle to LengthMeasurement (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of LengthMeasurement

if hObject.Value == 1
    set(hObject,'String',msg('LengthMeasurementButtonFinish'),'ForegroundColor','r','FontWeight','bold')
    hd = imdistline(handles.axes1);
    handles.hd = hd;
    guidata(hObject,handles)
    handles.UnitConversion.Enable = 'off';
    handles.DataCursorMode.Enable = 'off';
    handles.ShowTheoretical.Enable = 'off';
    handles.LoadAndTrim.Enable = 'off';
    handles.ExtractBall_RGB.Enable = 'off';
    handles.ExtractBall_HSV.Enable = 'off';
    handles.ExtractBall_LAB.Enable = 'off';
else
    set(hObject,'String',msg('LengthMeasurementButton'),'ForegroundColor','k','FontWeight','normal')
    hd = handles.hd;
    dist = getDistance(hd);
    set(handles.edit_length_pixel,'String',num2str(dist))
    delete(hd)
    handles.hd = [];
    guidata(hObject,handles)
    handles.UnitConversion.Enable = 'on';
    handles.DataCursorMode.Enable = 'on';
    handles.ShowTheoretical.Enable = 'on';
    handles.LoadAndTrim.Enable = 'on';
    handles.ExtractBall_RGB.Enable = 'on';
    handles.ExtractBall_HSV.Enable = 'on';
    handles.ExtractBall_LAB.Enable = 'on';
end



% --- Executes on button press in DataCursorMode.
function DataCursorMode_Callback(hObject, eventdata, handles)
% hObject    handle to DataCursorMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of DataCursorMode
if get(hObject,'Value')==1
    dcmObj = datacursormode(handles.figure1);
    set(dcmObj,'UpdateFcn',{@datacursorDisplayFcn,handles})
    set(dcmObj,'Enable','on')
    set(hObject,'String',msg('EndDataInspection'),'ForegroundColor','r','FontWeight','bold')
    handles.UnitConversion.Enable = 'off';
    handles.LengthMeasurement.Enable = 'off';
    handles.ShowTheoretical.Enable = 'off';
    handles.LoadAndTrim.Enable = 'off';
    handles.ExtractBall_RGB.Enable = 'off';
    handles.ExtractBall_HSV.Enable = 'off';
    handles.ExtractBall_LAB.Enable = 'off';

else
    dcmObj = datacursormode(handles.figure1);
    removeAllDataCursors(dcmObj)
    set(dcmObj,'Enable','off')
    set(hObject,'String',msg('StartDataInspection'),'ForegroundColor','k','FontWeight','normal')
    handles.UnitConversion.Enable = 'on';
    handles.LengthMeasurement.Enable = 'on';
    handles.ShowTheoretical.Enable = 'on';
    handles.LoadAndTrim.Enable = 'on';
    handles.ExtractBall_RGB.Enable = 'on';
    handles.ExtractBall_HSV.Enable = 'on';
    handles.ExtractBall_LAB.Enable = 'on';
end

function txt = datacursorDisplayFcn(obj,event_obj,handles)
% Customizes text of data tips
pos = get(event_obj,'Position');
switch get(get(event_obj,'Target'),'Parent')
    case handles.axes2
        txt = {[msg('DataCursorTime'),num2str(pos(1))],...
            ['x: ',num2str(pos(2))]};
    case handles.axes3
        txt = {[msg('DataCursorTime'),num2str(pos(1))],...
            ['y: ',num2str(pos(2))]};
    case handles.axes6
        txt = {['x: ',num2str(pos(1))],...
            ['y: ',num2str(pos(2))]};
end


function [xx, yy] = calcRangeData(minValDeg,maxValDeg)
th = 0:360;
r = 1;
if minValDeg <= maxValDeg
    id = th >= minValDeg & th <= maxValDeg;
else
    id = [find(th >= minValDeg) find(th <= maxValDeg)];
end

xx = r*cosd([minValDeg th(id) maxValDeg]);
yy = r*sind([minValDeg th(id) maxValDeg]);



% --- Executes on button press in LoadAndTrim.
function LoadAndTrim_Callback(hObject, eventdata, handles)
% hObject    handle to LoadAndTrim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

out = trim_video;
if isempty(out)
    return
end
handles.vidData = out{1};
handles.FrameRate = out{2};
handles.procVidData = zeros(size(handles.vidData),class(handles.vidData));
if ~isempty(handles.vidData)
    handles.hVideoImage = imshow(handles.vidData(:,:,:,1),'Parent',handles.axes1);
    handles.FrameSlider.Min = 1;
    handles.FrameSlider.Max = size(handles.vidData,4);
    handles.FrameSlider.Value = 1;
    handles.FrameSlider.SliderStep = [1/(handles.FrameSlider.Max-1) 10/(handles.FrameSlider.Max-1)];

    handles.isProcessed = false;
    
    cla(handles.axes2)
    cla(handles.axes3)
    cla(handles.axes6)
    
    handles.edit_length_pixel.String = '0';
    handles.edit_length_cm.String = '0';
    
    handles.LengthMeasurement.Enable = 'off';
    handles.UnitConversion.Enable = 'off';
    handles.DataCursorMode.Enable = 'off';
    
    handles.ShowTheoretical.Enable = 'off';

    handles.ExtractBall_RGB.Enable = 'on';
    handles.ExtractBall_HSV.Enable = 'on';
    handles.ExtractBall_LAB.Enable = 'on';
    
    handles.FrameSlider.Enable = 'on';
    
    % しきい値の初期化
    handles.RGB_Thresh = [0 255 0 255 0 255];
    handles.HSV_Thresh = [0 1];
    handles.LAB_Thresh = [-50 50];
    
    set(hObject,'FontWeight','normal','ForegroundColor','k')
    
    guidata(hObject, handles)
end

% --- Executes on slider movement.
function FrameSlider_Callback(hObject, eventdata, handles)
% hObject    handle to FrameSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

val = round(get(hObject,'Value'));
if handles.isProcessed
    handles.hVideoImage.CData = handles.procVidData(:,:,:,val);
else
    handles.hVideoImage.CData = handles.vidData(:,:,:,val);
end


% --- Executes during object creation, after setting all properties.
function FrameSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FrameSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in ExtractBall_RGB.
function ExtractBall_RGB_Callback(hObject, eventdata, handles)
% hObject    handle to ExtractBall_RGB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

thresh = RGB_Threshold_Tool(handles.vidData(:,:,:,round(handles.FrameSlider.Value)),handles.RGB_Thresh);
if isempty(thresh)
    return
end
handles.RGB_Thresh = thresh;
handles.colorType = 'rgb';

guidata(hObject,handles)

track_object(hObject,handles)


% --- Executes on button press in ExtractBall_HSV.
function ExtractBall_HSV_Callback(hObject, eventdata, handles)
% hObject    handle to ExtractBall_HSV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

thresh = HSV_Threshold_Tool(handles.vidData(:,:,:,round(handles.FrameSlider.Value)),handles.HSV_Thresh);
if isempty(thresh)
    return
end

handles.HSV_Thresh = thresh;
handles.colorType = 'hsv';

guidata(hObject,handles)

track_object(hObject,handles)


% --- Executes on button press in ExtractBall_LAB.
function ExtractBall_LAB_Callback(hObject, eventdata, handles)
% hObject    handle to ExtractBall_LAB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


thresh = LAB_Threshold_Tool(handles.vidData(:,:,:,round(handles.FrameSlider.Value)),handles.LAB_Thresh);
if isempty(thresh)
    return
end

handles.LAB_Thresh = thresh;
handles.colorType = 'lab';

guidata(hObject,handles)

track_object(hObject,handles)


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --------------------------------------------------------------------
function Language_Menu_Callback(hObject, eventdata, handles)
% hObject    handle to Language_Menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function LanguageItemsCallback(hObject, eventdata, handles)

set(handles.LanguageItems,'Checked','off');
hObject.Checked = 'on';
lang = hObject.UserData;
if ~isequal(getpref('ParabolaApp','lang'),lang)
    setpref('ParabolaApp','lang',lang)
    uiwait(msgbox(['Changed language setting to ',hObject.Text],'modal'))

    updateLanguage(handles)
end


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function updateLanguage(handles)

handles.figure1.Name = msg('figure1Name');
handles.LoadAndTrim.String = msg('LoadAndTrimButton');
handles.uipanel7.Title = msg('uipanel7Title');
handles.text13.String = msg('text13String');
handles.text12.String = msg('text12String');
handles.text2.String = msg('text2String');
handles.text9.String = msg('text9String');
handles.text4.String = msg('text4String');
handles.text5.String = msg('text5String');
handles.uipanel1.Title = msg('uipanel1Title');
handles.uipanel3.Title = msg('uipanel3Title');
handles.uipanel6.Title = msg('uipanel6Title');
handles.uipanel5.Title = msg('uipanel5Title');
handles.ShowTheoretical.String = msg('ShowTheoreticalButton');
handles.LengthMeasurement.String = msg('LengthMeasurementButton');
handles.UnitConversion.String = msg('UnitConversionButton');
handles.DataCursorMode.String = msg('DataCursorModeButton');

handles.LoadAndTrim.FontName = msg('FontName');
handles.uipanel7.FontName = msg('FontName');
handles.text13.FontName = msg('FontName');
handles.text12.FontName = msg('FontName');
handles.text2.FontName = msg('FontName');
handles.text9.FontName = msg('FontName');
handles.text4.FontName = msg('FontName');
handles.text5.FontName = msg('FontName');
handles.uipanel1.FontName = msg('FontName');
handles.uipanel4.FontName = msg('FontName');
handles.uipanel3.FontName = msg('FontName');
handles.uipanel6.FontName = msg('FontName');
handles.uipanel5.FontName = msg('FontName');
handles.ShowTheoretical.FontName = msg('FontName');
handles.LengthMeasurement.FontName = msg('FontName');
handles.UnitConversion.FontName = msg('FontName');
handles.DataCursorMode.FontName = msg('FontName');
