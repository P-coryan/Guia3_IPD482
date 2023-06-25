clear,
clc,
close all,

set(figure(),'WindowStyle','docked') % Insert the figure to dock
figure(1), hold on

xlim([-5 10]);
ylim([-5 10]);
global maxRange;

maxRange = 10; %Distancia m�xima del laser
% axis equal

pasoTiempo = 0.1; %Tiempo de Sampling de mi sistema

%Inicializo Robots
robot.x = 0;
robot.y = 0;    %es el robot segun odometria sin ruido (real)
robot.tita = 0;

robot2 = robot; %es el robot odometria con ruido 
%ambos robots empiezan en el mismo lugar

%Unico landmarks. Con tecla enter, finalizo ubicaci�n. Landmarks son
%guaradadas en matriz M, que contiene las posiciones reales [x,y] de cada
%landmark.
M = ginput();
plot(M(:,1),M(:,2),'ob'); %grafico landmarks
[Caras,Vertices] = conversionLandmarks(M);

%Genero el camino que quiero que siga el robot, entre las landmarks. El
%camino solo va de izquierda a derecha
[X,Y] = ginput;
xmax = max(X);
xmin = min(X);

xx = xmin:0.01:xmax;
yy = interp1(X,Y,xx,'spline');

CaminoReferencia = [xx' yy'];
plot(xx',yy','-r');

%%
%Inicializo Robots en el punto de partida del camino generado
% Robot Odometrico sin ruido:
robot.x = X(1);
robot.y = Y(1);
robot.tita = atan2(Y(2) - Y(1), X(2) - X(1)); %es la pose del robot seg�n la odometr�a. 
% Sin embargo, tener presente que el robot "cree" que tiene su pose sin error.
% Robot Odometrico con ruido:
robot2 = robot; % ambos robots empiezan en el mismo lugar

%Grafico ambos robots (odom�trico y real)
H1 = plotRobot(robot);      % Robot odometrico (sin ruido)
H2 = plotRobot2(robot2);    % Robot odometrico (con ruido)

% Inicializacion Nube de Puntos Global
nubePtos_1 = [];
nubePtos_2 = [];

% Genero un bucle para controlar el camino del robot
for cont = 2:length(xx)-1
    delete(H1);
    delete(H2);
    
    % ###########################   
    %Sensado de Postes (LASER)
    Laser = MedicionConLaserRapido(Vertices, Caras, robot2);    % Laser con robot2
    H3 = plotLaser(Laser,robot2);
    
    % Deteccion de Postes (NUBE DE PUNTOS) Robot odometrico sin error
    BuscoPostes_1 = [];
    BuscoPostes_1 = DeteccionPostes(Laser,robot);         
    nubePtos_1 = [nubePtos_1 ; BuscoPostes_1];
    
    % Deteccion de Postes (NUBE DE PUNTOS) Robot odometrico con error
    BuscoPostes_2 = [];
    BuscoPostes_2 = DeteccionPostes(Laser,robot2);         
    nubePtos_2 = [nubePtos_2 ; BuscoPostes_2];

    
    %##########################################%
    %## FILTRO KALMAN (ESTIMACION POSE ROBOT)##%
    %##########################################%
    if cont == 2 
        [xhat, P] = init_KalmanFilter(Laser, robot, M);   
    else
        [xhat, P] = FiltroKalman_Odometrico(Laser ,xhat ,P , V, W, pasoTiempo, cont);
    end
    robot_hat.x = xhat(1,cont);
    robot_hat.y = xhat(2,cont);
    robot_hat.tita = xhat(3,cont);
    
    
    
    %####### SEÑALES DE CONTROL u(t) DEL SISTEMA ##########%
    %referencias para el controlador (obviar esta parte) y obtenci�n de las
    %se�ales de control
    Q = .01*eye(3,3);
    H = [cos(robot.tita) 0;sin(robot.tita) 0; 0 1];
    xref = xx(cont);
    yref = yy(cont);
    titaref = atan2(yy(cont+1) - robot.y,(xx(cont+1) - robot.x));
    deltax = (xref-robot.x)/pasoTiempo + 0.1*(robot.x - xx(cont-1))/pasoTiempo;
    deltay = (yref-robot.y)/pasoTiempo + 0.3*(robot.y - yy(cont-1))/pasoTiempo;
    sigma1 = (Q(1,1))^2;
    sigma2 = (Q(2,2))^2;
    sigma3 = (Q(3,3))^2;
    Control(2) = (titaref-robot.tita)/pasoTiempo;
    Control(1) = (2*deltax*sigma2^2*cos(robot.tita))/(sigma2^2*cos(2*robot.tita) - sigma1^2*cos(2*robot.tita) + sigma1^2 + sigma2^2) + (2*deltay*sigma1^2*sin(robot.tita))/(sigma2^2*cos(2*robot.tita) - sigma1^2*cos(2*robot.tita) + sigma1^2 + sigma2^2);
    
    if Control(1) > 0.3
        Control(1) = 0.3;
    end
    if abs(Control(2)) > 1
        if (Control(2) <= 0)
            Control(2) = -1;
        else
            Control(2) = 1;
        end
    end
    V = Control(1);     % input u(t)
    W = Control(2);     % input u(t)
    
    %####### MODELOS DEL SISTEMA ##########%
    % Posicion del robot odometrico sin error
    robot.x = robot.x + pasoTiempo*V*cos(robot.tita);
    robot.y = robot.y + pasoTiempo*V*sin(robot.tita);
    robot.tita = robot.tita + pasoTiempo*W;
    
    % Posicion del robot odometrico con error
    robot2.x = robot2.x + pasoTiempo*V*cos(robot2.tita)+0.001*rand(1,1);
    robot2.y = robot2.y + pasoTiempo*V*sin(robot2.tita)+0.001*rand(1,1);
    robot2.tita = robot2.tita + pasoTiempo*W+0.01*rand(1,1)*W;
    
    %####### GRAFICAS ROBOTS ##########%
    H1=plotRobot(robot);        % (gris)
    H2=plotRobot2(robot2);      % (verde)
    H4=plotRobot_hat(robot_hat);% (rojo)
    H5=plotLaser_hat(Laser,robot_hat);  %(Laser hat Blue de linea continua)
   
%     pause(pasoTiempo);
    pause(1e-4);
    
    delete(H1);
    delete(H2);
    delete(H3);
    delete(H4);
    delete(H5);
end

% Datos.pasoTiempo = pasoTiempo;
% Datos.Laser = Laser;
% Datos.robot = robot;
% Datos.postes = M;
% Datos.nubePtos = nubePtos_1;
% save('Datos','Datos')



%% Pregunta 1 Robot Odometrico sin error
% Obtencion de Caracteristica M y su covarianza
[caractM_1, cov_caractM_1, error_caractM_1, label_caractM_1, idx_1] = ClusteringNube(nubePtos_1, M, 0.1, 4);

figure()
title('Clustering Nube (Robot Odometrico sin error)')
hold on, grid on
gscatter(nubePtos_1(:,1),nubePtos_1(:,2),idx_1);
plot( caractM_1(:,1) , caractM_1(:,2) ,'.k','MarkerSize', 25)   % caracteristica M estimada


%% Pregunta 2 Robot Odometrico con error
% idem anterior cambiando la deteccionPostes a robot 1 en Linea 79 codigo.
% Obtencion de Caracteristica M y su covarianza
[caractM_2, cov_caractM_2, error_caractM_2, label_caractM_2, idx_2] = ClusteringNube(nubePtos_2, M, 0.1, 4);

figure()
title('Clustering Nube (Robot Odometrico con error)')
hold on, grid on
gscatter(nubePtos_2(:,1),nubePtos_2(:,2),idx_2);
plot( caractM_2(:,1) , caractM_2(:,2) ,'.k','MarkerSize', 25)   % caracteristica M estimada


%% Pregunta 3 Localizacion (caracteritstias mapa conocidas)






%% Postes Globales
% figure
% title('Deteccion de Postes Eje global')
% hold on, grid on
% plot(M(:,1), M(:,2), 'ob')                                  % Postes verdaderos
% plot(nubePtos_1(:,1), nubePtos_1(:,2),'.r')                     % Nube de datos LiDar
% plot(Vertices(:,1), Vertices(:,2), '.k', 'MarkerSize', 15)  % Vertices verdaderos
% plot( caractM_1(:,1) , caractM_1(:,2) ,'.g','MarkerSize', 25)           % Postes estimados



