function varargout = moniter(varargin)
% MONITER MATLAB code for moniter.fig
%      MONITER, by itself, creates a new MONITER or raises the existing
%      singleton*.
%
%      H = MONITER returns the handle to a new MONITER or the handle to
%      the existing singleton*.
%
%      MONITER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MONITER.M with the given input arguments.
%
%      MONITER('Property','Value',...) creates a new MONITER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before moniter_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to moniter_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help moniter


gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @moniter_OpeningFcn, ...
    'gui_OutputFcn',  @moniter_OutputFcn, ...
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


% --- Executes just before moniter is made visible.
function moniter_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to moniter (see VARARGIN)


% UIWAIT makes moniter wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = moniter_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
%varargout{1} = handles.output;




% --------------------------------------------------------------------
function system_Callback(hObject, eventdata, handles)
% hObject    handle to system (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%
% --------------------------------------------------------------------
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2




% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
global i;
global k;
global T01;
global T02;
global T03;
global T1;
global area;

Imzero = zeros(420,560,3);      %计算图片背景
Im1 = rgb2gray(imread(['picture/','1.jpeg']));
%Im2 = rgb2gray(imread(['picture/','2.jpg']));
%Im3 = rgb2gray(imread(['picture/','3.jpg']));
%Im4 = rgb2gray(imread(['picture/','4.jpg']));
%Im5 = rgb2gray(imread(['picture/','5.jpg']));
%Imzero = (Im1)/5+(Im2)/5+(Im3)/5+(Im4)/5+(Im5)/5;
Imzero=Im1;
Imback = medfilt2(Imzero);
[MR,MC,Dim] = size(Imback);

%----卡尔曼滤波初始化----%
R  = [[0.2845,0.0045]',[0.0045,0.0455]'];        %观测噪声协方差矩阵
H  = [[1,0]',[0,1]',[0,0]',[0,0]'];              %观测转移矩阵
Q  = 0.01*eye(4);                                %状态噪声协方差矩阵（eye(4)指4*4单位阵）
P_init  = 100*eye(4);                                 %误差协方差
dt = 1;
A  = [[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];   %状态转移矩阵
g = 6;                      % pixels^2/time step
Bu = [0,0,0,g]';
kfinit=0;                   %初始指针
previous=0;
index=1;
%x=zeros(100,4);             %状态矩阵
x=zeros(400,4,100);
PK=zeros(400,16,100);

T1 = 50;
T2 = 0.7;
T3 = 0.7;

a1=str2num(get(handles.edit1,'String'));
a2=str2num(get(handles.edit2,'String'));
%起始默认坐标
X=0;Y=0;
str1=sprintf('后续思路\n\n');
str2=sprintf('           1.卡尔曼滤波，找到像素差异点\n');
str3=sprintf('           2.分割出前景，进行预测，得到人体框，通过框是否移动，以及长宽比例\n');   
str4=sprintf('           3.可以确定行走，站立，伸展手臂三种姿态。由于时间仓促，如需帮助，联系工程师Q：943601944！\n');
string=[str1 str2 str3 str4];
msgbox(string,'温馨提示','none');
return
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% 截图
SnapImage();
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
