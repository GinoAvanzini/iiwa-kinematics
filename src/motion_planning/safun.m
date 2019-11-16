function [sa] = safun(t,T,tau)
%%
%   Funcion es la derivada de ds  y son 2 escalones
%   Esta funcion se puede suavizar para tener jerk constante!!
%%
    a = 1 / (T * tau);
    if t < 0
        sa = 0;
    elseif t <= tau
        sa = a;
    elseif t> tau && t <= T
        sa = 0;
    elseif t>T && t<= (T+tau)
        sa = - a;
    elseif t> (T+tau)
        sa = 0;
    end
end