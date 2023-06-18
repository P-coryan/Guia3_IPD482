function [PMi, EDi, EPC, posteYgrupo, idx] = ClusteringNube(nubePtos, M)
    % Clustering postes Global total
    [idx, ~] = dbscan(nubePtos, 0.1, 4);  % (data, dist entre Vecinos, n�Min de Vecinos para formar grupo)

    % Analisis por grupo
    posteYgrupo = zeros(length(M(:,1)), 1); % |idx | poste |
%     pmPostes = zeros(length(idx), 2);
    PMi = zeros(length(M(:,1)), 2);

    % Error inicializacion
    EDi = ones(length(M(:,1)),1)*100;
    % Error por coordenada
    EPC = zeros(length(M(:,1)),2);
    % Verifica el largo del for si hay ruido del cluster (-1)
    if idx(idx <0)
        largoGrupos = length(unique(idx))-1;
    else
        largoGrupos = length(unique(idx));
    end
    
    for i = 1:largoGrupos
        % Separacion por grupo
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
        PMi(i,:) = (pGi(1,:) + pGi(2,:))/2;
%         pmPostes(i,:) = PMi(i,:);

        % Encuentra el poste con menos distancia
        posteID = 0;
        for j =1:length(M(:,1))
            if norm(M(j,:) -  PMi(i,:)) < EDi(i)
                EDi(i) = norm(M(j,:) - PMi(i,:));
                posteID = j;
            end
        end

        % Se agrupa el poste y el grupo que le corresponde
        posteYgrupo(i) = posteID;
        
        % Error de cada coordenada
        EPC(i,:) = PMi(i,:) - M(posteID,:);
        
%         plot(pGi(:,1), pGi(:,2) , '.-b','MarkerSize', 25)     % puntos mas extensos
%         plot( PMi(i,1) , PMi(i,2) ,'.k','MarkerSize', 25)     % punto medio estimado
%         plot(iVerticesPlot(:,1), iVerticesPlot(:,2),'k')
    end
end

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

% figure()
% hold on
% plot(grupo1(:,1), grupo1(:,2),'.m')
% plot(verticesGrupo1plot(:,1), verticesGrupo1plot(:,2),'g')
% 
% plot(pG1(:,1), pG1(:,2) , '.-b','MarkerSize', 25)
% plot( pmG1(1) , pmG1(2) ,'.r','MarkerSize', 25)
% plot(M(posteNro,1), M(posteNro,2), 'ob')
% 