function [PMi, covNP, EPC, posteYgrupo, idx] = ClusteringNube(nubePtos, M, distCluster, cantCluster)
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

    % Inicializacion error para clasificacion grupo 
    EDi = ones(largoGrupos,1)*100;
    
    % Inicializacion Error caracteristica (valor estimado con valor real)
    EPC = zeros(largoGrupos,2);
   
    % Inicializacion Covarianza de postes 
    covNP= cell(largoGrupos,1);
    
   
    for i = 1:largoGrupos
        % Separacion por grupo
        iGrupo = nubePtos(idx == i, :);
%         iVerticesPlot = iGrupo(convhull(iGrupo), :);
        iVertices = unique(iGrupo(convhull(iGrupo), :), 'rows'); % borra punto final = punto inicial 

        % Distancia entre vertices del grupo
        distGi = zeros(length(iVertices));
        for j=1:length(iVertices)
            for k=1:length(iVertices)
                distGi(j,k) = norm(iVertices(j,:) - iVertices(k,:));
            end
        end

        % Encuentra distancia maxima entre vertices del grupo
        [a, ~] = find(distGi == max(max(distGi)));
        pGi = [iVertices(a(1),:); iVertices(a(2),:)];
        PMi(i,:) = (pGi(1,:) + pGi(2,:))/2;              % poste estimado

        % Encuentra el poste con menos distancia (Para analizar el Error)
        posteID = 0;
        for j =1:length(M(:,1))
            if norm(M(j,:) -  PMi(i,:)) < EDi(i)
                EDi(i) = norm(M(j,:) - PMi(i,:));
                posteID = j;
            end
        end
        posteYgrupo(i) = posteID;           % etiqueta del poste estimado
        
        % Error de cada coordenada del poste estimado
        EPC(i,:) = abs(PMi(i,:) - M(posteID,:));
        
        % Obtencion Matriz Covarianza de cada caracteristica estimada
        vX = var(nubePtos( idx==i , 1));
        vY = var(nubePtos( idx==i , 2));
        covNP{i} = [vX 0; 0 vY];
        
        % % M
        % sizeM = length(caractM(:,1));
        % CP = cell(sizeM,1);
        % for i=1:sizeM
        %     eX = abs(errorCaractM(i, 1));
        %     eY = abs(errorCaractM(i, 2));
        %     CP{i} = [eX 0; 0 eY];
        % end
        
        
    
%         plot(pGi(:,1), pGi(:,2) , '.-b','MarkerSize', 25)     % puntos mas extensos
%         plot( PMi(i,1) , PMi(i,2) ,'.k','MarkerSize', 25)     % punto medio estimado
%         plot(iVertices(:,1), iVertices(:,2),'k')
    
    end
end

