function x = DeteccionPostes(Laser,robot)
    
    % Matrices de traslacion y rotacion global
    T = @(tx,ty) [1 0 tx; 0 1 ty; 0 0 1];
    R = @(tita) [cos(tita) -sin(tita); sin(tita) cos(tita)];
    
    % Filtraje por Radio r < Rmax
    rangoRadio = 9.5;
    candidatos = Laser( : , (Laser(1,:) < rangoRadio));
    globalCandidates = zeros(length(candidatos(1,:)), 2);
    
    if ~isempty(candidatos)
        % Angulos a radianes y puntos a cartesianos
        candidatos(2,:) = deg2rad(candidatos(2,:)) - pi/2; %angulo grados a radianes
        [candX, candY] = pol2cart(candidatos(2,:), candidatos(1,:));    % ptos cartesianos
        candidatos = [candX ; candY];
        
        % Matrices de rotacion y traslacion para puntos globales finales
        for i=1:length(candidatos(1,:))
            newCord = R(robot.tita) * [candidatos(1,i);candidatos(2,i)];
            newCord = T(robot.x, robot.y) * [newCord; 1]; 

            globalCandidates(i , :) = newCord(1:2);
        end
    end
    
    x = globalCandidates;
    
    
%     al parecer DeteccionPostes es en coordenadas locales del robot
%    pasar primero a global y despues analizar para no realizar el mismo
%    calculo en caso de que el lidar vea el mismo poste desde otro punto de
%    vista

%     -> r < Rmax
%     -> robot.tita y angulo(r < Rmax)
%     -> agregarlos a una matriz
%     -> aplicar la matriz de rotacion y traslacion hacia el eje global
%     - robot.x, robot.y y pts
%     -> Se obtiene nube de puntos con los postes en el eje global
     
end