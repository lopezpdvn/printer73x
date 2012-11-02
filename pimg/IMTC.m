function varargout = IMTC(varargin)
% IMTC M-file for IMTC.fig
%      IMTC, by itself, creates a new IMTC or raises the existing
%      singleton*.
%
%      H = IMTC returns the handle to a new IMTC or the handle to
%      the existing singleton*.
%
%      IMTC('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IMTC.M with the given input arguments.
%
%      IMTC('Property','Value',...) creates a new IMTC or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before IMTC_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to IMTC_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help IMTC

% Last Modified by GUIDE v2.5 13-Oct-2011 23:34:43

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @IMTC_OpeningFcn, ...
                   'gui_OutputFcn',  @IMTC_OutputFcn, ...
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

% --- Executes just before IMTC is made visible.
function IMTC_OpeningFcn(hObject, eventdata, handles, varargin)
% Colocar Imagen de Matlab
T = imread('Matlab.jpg'); 
axes(handles.axes5);
axis off;
imshow(T); 
handles.output = hObject;
guidata(hObject, handles);
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to IMTC (see VARARGIN)

% Choose default command line output for IMTC
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes IMTC wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = IMTC_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;

% Declaración de parámetros para la detección de bordes
     Nx1 = 10; Sigmax1 = 1; Nx2 = 10; Sigmax2 = 1; Theta1 = pi/2;
     Ny1 = 10; Sigmay1 = 1; Ny2 = 10; Sigmay2 = 1; Theta2 = 0;
     alfa = 0.1; RR = 0; CC = 1;

if CC == 1
    warndlg('Proceso en curso','Algoritmo de Canny');
end
     
% Se lee la imagen a procesar
% x = imread('Edge.jpg');
x = imread('Edge.png');
w = rgb2gray(x);
% figure(1); 
colormap(gray);
% subplot(1,2,1);
% imagesc(w);
% title('Escala de grises');
% imwrite(w,'Grises.jpg')
imwrite(w,'Grises.png')

% Detección de bordes en el eje X
% subplot(3,2,2);
filterx = d2dgauss(Nx1,Sigmax1,Nx2,Sigmax2,Theta1);
Ix = conv2(w,filterx,'same');
% imagesc(Ix);
% title('Ix');

% Detección de bordes en el eje Y
% subplot(3,2,3)
filtery = d2dgauss(Ny1,Sigmay1,Ny2,Sigmay2,Theta2);
Iy = conv2(w,filtery,'same'); 
% imagesc(Iy);
% title('Iy');

% Modelo del gradiente - Combinación de los ejes XY
% subplot(3,2,4);
NVI = sqrt(Ix.*Ix+Iy.*Iy);
% imagesc(NVI);
% title('Norm of Gradient');

% Histéresis de umbral
I_max = max(max(NVI));
I_min = min(min(NVI));
level = alfa*(I_max-I_min)+I_min;
% subplot(3,2,5);
Ibw = max(NVI,level.*ones(size(NVI)));
% imagesc(Ibw);
% title('After Thresholding');

% Cierre de contornos abiertos
[n,m] = size(Ibw);
for i = 2:n-1,
for j = 2:m-1,
	if Ibw(i,j) > level,
	X = [-1,0,+1;-1,0,+1;-1,0,+1];
	Y = [-1,-1,-1;0,0,0;+1,+1,+1];
	Z = [Ibw(i-1,j-1),Ibw(i-1,j),Ibw(i-1,j+1);
	   Ibw(i,j-1),Ibw(i,j),Ibw(i,j+1);
	   Ibw(i+1,j-1),Ibw(i+1,j),Ibw(i+1,j+1)];
	XI = [Ix(i,j)/NVI(i,j), -Ix(i,j)/NVI(i,j)];
	YI = [Iy(i,j)/NVI(i,j), -Iy(i,j)/NVI(i,j)];
	ZI = interp2(X,Y,Z,XI,YI);
		if Ibw(i,j) >= ZI(1) & Ibw(i,j) >= ZI(2)
		I_temp(i,j) = I_max;
		else
		I_temp(i,j) = I_min;
		end
	else
	I_temp(i,j) = I_min;
	end
end
end
colormap(gray);
% imwrite(I_temp,'Bordes.jpg');
imwrite(I_temp,'Bordes.png');
RR = 1;

if RR == 1
    msgbox('Proceso terminado','Algoritmo de Canny');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function pushbutton1_CreateFcn(hObject, eventdata, handles)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Función "d2dgauss"
% Esta función regresa un detector de bordes 2D de tamaño n1xn2, theta es 
% el angulo de rotación del detector y sigma1 y sigma2 son las desviaciones
% estandar de las funciones gaussianas.
function h = d2dgauss(n1,sigma1,n2,sigma2,theta)
r = [cos(theta) -sin(theta);
     sin(theta)  cos(theta)];
for i = 1:n2 
    for j = 1:n1
        u = r*[j-(n1+1)/2 i-(n2+1)/2]';
        h(i,j) = gauss(u(1),sigma1)*dgauss(u(2),sigma2);
    end
end
h = h/sqrt(sum(sum(abs(h).*abs(h))));

% Función "gauss"
function y = gauss(x,std)
y = exp(-x^2/(2*std^2))/(std*sqrt(2*pi));

% Función "dgauss" - Derivada de primer orden de la función gaussiana
function y = dgauss(x,std)
y = -x*gauss(x,std)/std^2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% Abrimos el archivo a procesar
% [FileName Path] = uigetfile({'*.jpg;*.bmp'},'Abrir Imagen');
[FileName Path] = uigetfile({'*.png'},'Abrir Imagen');
if isequal(FileName,0)
return
else
axes(handles.axes2)    
a = imread(strcat(Path,FileName));
axis off
imshow(a);
end
handles.direccion = strcat(Path,FileName);
guidata(hObject,handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% Obtenemos la imagen del axes
rgb = getimage(handles.axes2); 
if isempty(rgb), return, end 
% Guardamos el archivo
% formatos = {'*.jpg','JPEG (*.jpg)';'*.tif','TIFF (*.tif)'}; 
formatos = {'*.png','PNG (*.png)'}; 
[nomb,ruta] = uiputfile(formatos,'Guardar Imagen'); 
if nomb == 0, return, end 
fName = fullfile(ruta,nomb); 
imwrite(rgb,fName); 

% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
%--------------------------------------------------------%
axes(handles.axes3)
xx = imread('Grises.png'); 
colormap(gray);
axis off;
imagesc(xx);
% title('Escala de grises');
%--------------------------------------------------------%
axes(handles.axes4)
yy = imread('Bordes.png'); 
colormap(gray);
axis off;
imagesc(yy);
% title('Imagen procesada');
%--------------------------------------------------------%
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% Obtenemos la imagen del axes
rgb = getimage(handles.axes4); 
if isempty(rgb), return, end 
% Guardamos el archivo
% formatos = {'*.jpg','JPEG (*.jpg)';'*.tif','TIFF (*.tif)'};
formatos = {'*.png','PNG (*.png)'};
[nomb,ruta] = uiputfile(formatos,'Guardar Imagen'); 
if nomb == 0, return, end 
fName = fullfile(ruta,nomb); 
imwrite(rgb,fName); 
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
