# Pregunta 3

## Introducción: Filtro de Kalman Extendido

El filtro de Kalman extendido (EKF, por sus siglas en inglés) es una extensión no lineal del filtro de Kalman, utilizado para estimar el estado de un sistema no lineal a partir de mediciones que contienen ruido. EKF combina la idea de propagación de la media y la covarianza de un filtro de Kalman junto con la linealización de un sistema no lineal a través de una expansión de Taylor de primer orden.

En el contexto de un robot con un LIDAR 2D que necesita localizarse, el EKF es de gran utilidad. El mapa M del entorno del robot es conocido y consta de una serie de características que el LIDAR puede detectar. El robot utiliza el EKF para fusionar la información de su sistema de navegación inercial y las mediciones del LIDAR para obtener una estimación precisa de su ubicación.

El filtro se basa en dos ecuaciones clave: 

- **La ecuación de actualización:** Actualiza la estimación del estado y la covarianza en función de la medición actual.
- **La ecuación de predicción:** Predice la próxima estimación del estado y la covarianza basándose en el estado actual.

## Simulación de Prueba

A continuación se muestra una simulación que soluciona la pregunta. En esta simulación, inicializamos la matriz de covarianza P con el vector P = diag( [ ones(1,2), 0.1, zeros(1,length(M))] ); La matriz de covarianza se utiliza para expresar nuestras incertidumbres sobre la estimación del estado  y el vector estado representa la pose  del robot  y las caracteriticas puntuales del mapa,  se inicializa como xhat = zeros( 3+length(M) ,2);  para su dimensión y los datos son xhat(:,2) = [robot.x ; robot.y ; robot.tita ; M];    .

![Simulación Robot](p3.gif)
