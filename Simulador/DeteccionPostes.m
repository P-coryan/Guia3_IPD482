function x = DeteccionPostes(Laser,robot)
    
    % Filtraje por Radio r < Rmax
    rangoRadio = 9.5;
    candidatos = Laser( : , (Laser(1,:) < rangoRadio));
    candidatos(2,:) = deg2rad(candidatos(2,:)); %angulo grados a radianes
    [candX, candY] = pol2cart(candidatos(2,:), candidatos(1,:));
    candidatosCart = [candX ; candY];
    
    
    plot(candX, candY, '.r')
    
    % Clustering
    [idx, isnoise] = dbscan(candidatosCart, 0.01, 10);
    
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
    

    



    x = 0;
    
    
    
    
    
    
end