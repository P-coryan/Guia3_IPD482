
function jH  = J1(Pref,r)
x=r.x;
y=r.y;
jH = zeros(2,3);
Delta(1) = (Pref(1) - x);
Delta(2) = (Pref(2) - y);
r = norm(Delta);
jH(1,1) = -Delta(1) / r;
jH(1,2) = -Delta(2) / r;
jH(2,1) = Delta(2) / (r^2);
jH(2,2) = -Delta(1) / (r^2);
jH(2,3) = -1;
end