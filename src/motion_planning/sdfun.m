function [sd] = sdfun(t,T,tau)
%%
%   Funcion es la derivada de s  y es un trapecio
%   Esta funcion se puede suavizar para tener jerk constante!!
%%
    a = 1 / (T * tau);
    v = 1 / T;
    if t < 0
        sd = 0;
    elseif t <= tau
        sd = a * t;
    elseif t> tau && t <= T
        sd = v;
    elseif t>T && t<= (T+tau)
        sd = v - a * (t-T);
    elseif t> (T+tau)
        sd = 0;
    end
end