function [c,ceq] = solarSailConstraints(xF,p)

c = [];

rF = xF(1:3);
vF = xF(4:6);

ceq = [norm(rF) - p.r_final;
    norm(vF) - sqrt(p.mu/p.r_final)];

end