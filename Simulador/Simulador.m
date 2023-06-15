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
% Clustering postes Global total
[idx, isnoise] = dbscan(nubePtos, 0.1, 4);  % (data, dist entre Vecinos, n�Min de Vecinos para formar grupo)
figure()
gscatter(nubePtos(:,1),nubePtos(:,2),idx);

%######### Analisis Grupo 1
% grupo1 = nubePtos( idx==1 , :);
% verticesGrupo1plot = grupo1(convhull(grupo1), :);
% verticesGrupo1 = unique(grupo1(convhull(grupo1), :), 'rows');   % delete punto inicial=punto final

% Distancia entre vertices Grupo 1
% distGrupo1 = zeros(length(verticesGrupo1));
% for i=1:length(verticesGrupo1)
%     for j=1:length(verticesGrupo1)
%        distGrupo1(i,j) = norm(verticesGrupo1(i,:) - verticesGrupo1(j,:));
%     end
% end
% % Distancia maxima de vertices Grupo 1
% [ a,~ ] = find( distGrupo1== max(max(distGrupo1)) ); 
% % Puntos vertices Grupo 1 con mayor distancia
% pG1 = [verticesGrupo1(a(1),:) ; verticesGrupo1(a(2),:)];
% % Punto medio del Grupo 1 (posicion del poste)
% pmG1 = (pG1(1,:) + pG1(2,:))/2;
% 
% % Error distancia Grupo 1 (analiza todos los postes para encontrar a su original)
% error_pmG1 = 100; %  norm( M(1,:) - pmG1 );
% for i = 1:length(M(:,1))
%     if norm( M(i,:) - pmG1 ) < error_pmG1 
%         error_pmG1 = norm( M(i,:) - pmG1 );
%         posteNro = i;
%     end
% end


% Analisis por grupo
idx(idx <0) = []; % elimina idx = -1

posteYgrupo = zeros(length(M(:,1)), 1); % |idx | poste |
pmPostes = zeros(length(idx), 2);

hold on
for i = 1:length(unique(idx))
    iGrupo = nubePtos(idx == i, :);
    iVerticesPlot = iGrupo(convhull(iGrupo), :);
    iVertices = unique(iGrupo(convhull(iGrupo), :), 'rows'); % borra punto final = punto inicial 

    % Distancia entre vertices
    distGi = zeros(length(iVertices));
    for j=1:length(iVertices)
        for k=1:length(iVertices)
            distGi(j,k) = norm(iVertices(j,:) - iVertices(k,:));
        end
    end
    
    % Encuentra distancia maxima entre vertices
    [a, ~] = find(distGi == max(max(distGi)));
    pGi = [iVertices(a(1),:); iVertices(a(2),:)];
    PMi = (pGi(1,:) + pGi(2,:))/2;
    pmPostes(i,:) = PMi;
    
    % Encuentra el poste con menos distancia
    EDi = 100;
    posteID = 0;
    for j =1:length(M(:,1))
        if norm(M(i,:) - PMi) < EDi
            EDi = norm(M(i,:) - PMi);
            posteID = j;
        end
    end

    % Se agrupa el poste y el grupo que le corresponde
    posteYgrupo(i) = posteID;

%     plot(pGi(:,1), pGi(:,2) , '.-b','MarkerSize', 25)
    plot( PMi(1) , PMi(2) ,'.r','MarkerSize', 25)
%     pause(1);
end




figure()
hold on
plot(grupo1(:,1), grupo1(:,2),'.m')
plot(verticesGrupo1plot(:,1), verticesGrupo1plot(:,2),'g')

plot(pG1(:,1), pG1(:,2) , '.-b','MarkerSize', 25)
plot( pmG1(1) , pmG1(2) ,'.r','MarkerSize', 25)
plot(M(posteNro,1), M(posteNro,2), 'ob')



%% Postes Globales
figure
title('Deteccion de Postes Eje global')
plot(M(:,1), M(:,2), 'ob')
hold on
plot(nubePtos(:,1), nubePtos(:,2),'.r')
for i = 1:length(Vertices(:,1))
    plot(Vertices(i,1), Vertices(i,2), '.g', 'MarkerSize', 25)
    plot(Vertices(i,1), Vertices(i,2), '.g', 'MarkerSize', 25)
    plot(Vertices(i,1), Vertices(i,2), '.g', 'MarkerSize', 25)
    plot(Vertices(i,1), Vertices(i,2), '.g', 'MarkerSize', 25)
end

