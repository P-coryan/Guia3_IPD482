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
robot.y = 0; %es el robot segun odometria
robot.tita = 0;

robot2 = robot; %es el robot real
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
robot.x = X(1);
robot.y = Y(1);
robot.tita = atan2(Y(2) - Y(1), X(2) - X(1)); %es la pose del robot seg�n la odometr�a. 
% Sin embargo, tener presente que el robot "cree" que tiene su pose sin error.

robot2 = robot; %es el robot real
%ambos robots empiezan en el mismo lugar

%Grafico ambos robots (odom�trico y real)
H1 = plotRobot(robot);      % Robot odometrico
H2 = plotRobot2(robot2);    % Robot real

% Inicializacion Nube de Puntos Global
nubePtos = [];

% Genero un bucle para controlar el camino del robot
for cont = 2:length(xx)-1
    delete(H1);
    delete(H2);
    
    %Sensado de Postes
    Laser = MedicionConLaserRapido(Vertices, Caras, robot2);
    H3 = plotLaser(Laser,robot2);
    
    % Deteccion de Postes
    BuscoPostes = [];
    BuscoPostes = DeteccionPostes(Laser,robot2); %ojo, el robot "cree" que tiene la pose correcta
    nubePtos = [nubePtos ; BuscoPostes];
    
    
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
    V = Control(1);
    W = Control(2);
    
    % Posicion del robot (sin ruido)
    robot.x = robot.x + pasoTiempo*V*cos(robot.tita);
    robot.y = robot.y + pasoTiempo*V*sin(robot.tita);
    robot.tita = robot.tita + pasoTiempo*W;
    
    % Posicion del robot real (con ruido)
    robot2.x = robot2.x + pasoTiempo*V*cos(robot2.tita)+0.001*rand(1,1);
    robot2.y = robot2.y + pasoTiempo*V*sin(robot2.tita)+0.001*rand(1,1);
    robot2.tita = robot2.tita + pasoTiempo*W+0.01*rand(1,1)*W;
    
    H1=plotRobot(robot);
    H2=plotRobot2(robot2);
%     pause(pasoTiempo);
    pause(1e-4);
    delete(H1);
    delete(H2);
    delete(H3);
end

%%

[puntoMedioPoste, errorPoste ,iVerticesPlot, idx] = ClusteringNube(nubePtos, M);


figure()
title('Clustering Nube')
hold on, grid on
gscatter(nubePtos(:,1),nubePtos(:,2),idx);
plot( puntoMedioPoste(:,1) , puntoMedioPoste(:,2) ,'.k','MarkerSize', 25)   % punto medio estimado
plot(iVerticesPlot(:,1), iVerticesPlot(:,2),'k')


%% Postes Globales
figure
title('Deteccion de Postes Eje global')
hold on, grid on
plot(M(:,1), M(:,2), 'ob')                                  % Postes verdaderos
plot(nubePtos(:,1), nubePtos(:,2),'.r')                     % Nube de datos LiDar
plot(Vertices(:,1), Vertices(:,2), '.k', 'MarkerSize', 15)  % Vertices verdaderos
plot( puntoMedioPoste(:,1) , puntoMedioPoste(:,2) ,'.g','MarkerSize', 25)           % Postes estimados
plot(iVerticesPlot(:,1), iVerticesPlot(:,2),'k')            % Vertices estimados



