function [PMi, covNP, posteYgrupo, idx] = ClusteringKalman(nubePtos, distCluster, cantCluster)
    % Clustering postes Global total
    [idx, ~] = dbscan(nubePtos, distCluster, cantCluster);  % (data, dist max entre Vecinos, n�Min de Vecinos para formar grupo)

    % Largo de Clustering (si hay ruido del cluster (-1), no lo considera)
    if idx(idx <0)
        largoGrupos = length(unique(idx))-1;
    else
        largoGrupos = length(unique(idx));
    end
    
    % Analisis punto medio por grupo y clasificacion
    PMi = zeros(largoGrupos, 2);
    posteYgrupo = zeros(largoGrupos, 1);
    
    % Inicializacion Covarianza de postes 
    covNP= cell(largoGrupos,1);
    
   
    for i = 1:largoGrupos
        % Separacion por grupo
        iGrupo = nubePtos(idx == i, :);
        
        % Promedio de puntos para punto final del poste
        promX = mean(iGrupo(:,1));
        promY = mean(iGrupo(:,2));
        PMi(i,:) = [promX, promY];
        
        % Obtencion Matriz Covarianza de cada caracteristica estimada
        vX = var(nubePtos( idx==i , 1));
        vY = var(nubePtos( idx==i , 2));
        covNP{i} = [vX 0; 0 vY];

    end
end