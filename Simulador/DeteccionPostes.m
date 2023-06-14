function x = DeteccionPostes(Laser,robot)
    
    % Filtraje por Radio r < Rmax
    T = @(tx,ty) [1 0 tx; 0 1 ty; 0 0 1];
    R = @(tita) [cos(tita) -sin(tita); sin(tita) cos(tita)];

    rangoRadio = 9.5;
    candidatos = Laser( : , (Laser(1,:) < rangoRadio));
    candidatos(2,:) = deg2rad(candidatos(2,:)) - pi/2; %angulo grados a radianes
    [candX, candY] = pol2cart(candidatos(2,:), candidatos(1,:));
    candidatos = [candX ; candY];
    globalCandidates = zeros(2, length(candidatos));
    
    if ~isempty(candidatos)
        for i=1:length(candidatos)
            newCord = R(robot.tita) * [candidatos(1,i);candidatos(2,i)];
            newCord = T(robot.x, robot.y) * [newCord; 1]; 

            globalCandidates(:, i) = newCord(1:2);
        end
    end
%     figure()
%     subplot 121
%     plot(candX, candY, '.r')
%     subplot 122
%     plot(globalCandidates(1,:), globalCandidates(2,:), '.r')

    % Clustering
%     [idx, isnoise] = dbscan(candidatos, 0.01, 10);
    
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
    

    


    x = globalCandidates;
    
    
    
    
    
    
end