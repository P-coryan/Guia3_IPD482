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


# Pregunta 4

## Introducción: Filtro de Kalman Extendido con Características Desconocidas

Cuando no conocemos las características M del entorno del robot, el problema se vuelve un poco más complejo. En este caso, se debe realizar la construcción de mapas a la vez que se estima la localización del robot. Esto es lo que se conoce como el problema de localización y mapeo simultáneos, o SLAM (Simultaneous Localization and Mapping) en inglés.

El filtro de Kalman extendido (EKF) se puede utilizar para resolver el problema SLAM al estimar tanto la trayectoria del robot como la ubicación de las características en el mapa. Al igual que en el caso anterior, el robot utiliza el EKF para fusionar la información de su sistema de navegación inercial y las mediciones del LIDAR. Sin embargo, en este caso, las características se van agregando a medida que el robot avanza y descubre nuevas características.

Es importante destacar que para resolver el problema SLAM, se necesita un modelo de movimiento para el robot y un modelo de medición para el sensor.

## Simulaciones de Prueba

A continuación, se presentan dos simulaciones, la primera simulación se realiza con odometría pura y la segunda con odometría ruidosa.

### Simulación con Odometría Pura

En esta simulación, se supone que las mediciones de odometría son perfectas y no contienen ningún ruido. En la realidad, esto es muy raro, pero nos proporciona un punto de referencia útil.

![Simulación Odometría Pura](p4_puro.gif)

### Simulación con Odometría Ruidosa

En esta simulación, introducimos ruido en las mediciones de odometría para imitar la realidad más de cerca. Como resultado, el robot tiene que lidiar con las incertidumbres en sus mediciones y hacer un uso inteligente de su sensor LIDAR para localizarse y mapear su entorno.

![Simulación Odometría Ruidosa](p4_ruidoso.gif)

