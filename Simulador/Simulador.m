clear,
clc,
close all,
%Load camino guardado en path
load('DataMap.mat')
M = DataMap.M;
X = DataMap.X;
Y = DataMap.Y;

set(figure(),'WindowStyle','docked') % Insert the figure to dock
figure(1), hold on
title('"Trayectoria robot estimada"')
xlim([-5 10]);
ylim([-10 10]);

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
% M = ginput();
plot(M(:,1),M(:,2),'ob'); %grafico landmarks
[Caras,Vertices] = conversionLandmarks(M);

%Genero el camino que quiero que siga el robot, entre las landmarks. El
%camino solo va de izquierda a derecha
% [X,Y] = ginput;
xmax = max(X);
xmin = min(X);

xx = xmin:0.01:xmax;
yy = interp1(X,Y,xx,'spline');

CaminoReferencia = [xx' yy'];
plot(xx',yy','-r');

%% Save camino
% DataMap.M = M;
% DataMap.X = X;
% DataMap.Y = Y;
% save('DataMap','DataMap')

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
H1 = plotRobot(robot);      % Robot odometrico (sin ruido) (gris)
H2 = plotRobot2(robot2);    % Robot odometrico (con ruido) (verde)

% Inicializacion Nube de Puntos Global
nubePtos_1 = [];
nubePtos_2 = [];
    
pause(0.5)
delete(H1);
delete(H2);

% Genero un bucle para controlar el camino del robot
for cont = 2:length(xx)-1

    % ###########################   
    %Sensado de Postes (LASER)  con robot2
    Laser = MedicionConLaserRapido(Vertices, Caras, robot2);    
%     H3 = plotLaser(Laser,robot2);
    
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
    % (1== SI conocemos las caract)(0== NO conocemos las caract)
    knowMap = 0;
    if cont == 2 
        [xhat, P] = init_KalmanFilter(Laser, robot, M, knowMap);    
    else
        % [xhat, P] = FiltroKalman_version1(Laser ,xhat ,P , V, W, pasoTiempo, cont);
        [xhat, P] = FiltroKalman_Odometric(Laser ,xhat ,P , V, W, pasoTiempo, cont, knowMap);
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
    H1=plotRobot(robot);            % robot sin ruido(gris)
    H2=plotRobot2(robot2);          % robot ruidoso (verde)
    H3 = plotLaser(Laser,robot2);   % Laser
    H4=plotRobot_hat(robot_hat);    % (rojo)
%     H5=plotLaser_hat(Laser,robot_hat);  %(Laser_hat Blue de linea continua)
   
%     pause(pasoTiempo);
    pause(1e-5);
    
    delete(H1);
    delete(H2);
    delete(H3);
    delete(H4);
%     delete(H5);
end



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


%% Pregunta 3 y 4

% comparacion puntos con los estimados
caract_hatFinal = xhat(4:end,end);
for t = 1:length(caract_hatFinal)/2
    index = 2*(t-1) + 1;
    plot_caractHat(t,:) = [caract_hatFinal(index),caract_hatFinal(index+1)];
end

figure()
subplot 221 
title('Caracteristicas con robot 1') 
hold on
plot(M(:,1),M(:,2),'ob', 'MarkerSize',15); %grafico landmarks
plot( plot_caractHat(:,1), plot_caractHat(:,2), '.r'  , 'MarkerSize', 15, 'DisplayName', 'estimacion')
plot(nubePtos_1(:,1), nubePtos_1(:,2),'.g')
legend('caracteristicas', 'estimacion EKF', 'nube de puntos recopilada')

subplot 222
title('Caracteristicas con robot 2') 
hold on
plot(M(:,1),M(:,2),'ob', 'MarkerSize',15); %grafico landmarks
plot( plot_caractHat(:,1), plot_caractHat(:,2), '.r'  , 'MarkerSize', 15, 'DisplayName', 'estimacion')
plot(nubePtos_2(:,1), nubePtos_2(:,2),'.g')
legend('caracteristicas', 'estimacion EKF', 'nube de puntos recopilada')

subplot 223
title('Procesamiento Nube de puntos (robot 1)') 
hold on
plot(M(:,1),M(:,2),'ob', 'MarkerSize',15); %grafico landmarks
plot( caractM_1(:,1) , caractM_1(:,2) ,'.g','MarkerSize', 20)           % Postes estimados
legend('caracteristicas', 'clustering Nube de puntos')

subplot 224
title('Procesamiento Nube de puntos (robot 2)')
hold on
plot(M(:,1),M(:,2),'ob', 'MarkerSize',15); %grafico landmarks
plot( caractM_2(:,1) , caractM_2(:,2) ,'.g','MarkerSize', 20)           % Postes estimados
legend('caracteristicas', 'clustering Nube de puntos')


