
function Jac  = F1(v,theta)

s1 = sin(theta);
c1 = cos(theta);

Jac  = [1 0 -v*0.3*s1;
        0 1 v*0.3*c1;
        0 0 1];
end