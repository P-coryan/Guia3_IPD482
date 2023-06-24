
function [dHx, h_k, g, J1, J2] = matrixData()
    dHx = @(xi,yi,r,robot) [  (xi-robot.x)/r , (yi-robot.y)/r , 0 ;
                     (yi-robot.y)/r^2 , -(xi-robot.x)/r^2 , -1  ];

    % Transforma caracteristica global a local
    h_k = @(caract,robot) [ sqrt((caract(1,1)-robot.x)^2 + (caract(1,2)-robot.y)^2) ;
            rad2deg( atan2((caract(1,2)-robot.y),(caract(1,1)-robot.x)) -robot.tita + pi/2)];

    g = @(r,tita,robot) [robot.x+r*cos(tita+robot.tita) ; 
                        robot.y+r*sin(tita+robot.tita)];

    % dGz = @() [cos(tita + robot.tita) , -r*sen(tita+robot.tita)  ; 
    %         sin(tita + robot.tita) ,  r*cos(tita+robot.tita) ];

    % dYxz = @(n,r,tita,robot) [eye(n), zeros(n,2) ; zeros(2,n), dGz];

    J1 = @(v,theta,tau) [1 0 -v*tau*sin(theta);
                         0 1 v*tau*cos(theta);
                         0 0 1];
    J2 = @(theta,tau) [tau*cos(theta) 0;
                         tau*sin(theta) 0 ;
                         0  1];
end 