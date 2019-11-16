function [s] = sfun(t,T,tau)
%%
%   Funcion que hace que la derivada de s sea un trapecio
%   Esta funcion se puede suavizar para tener jerk constante!!
%%
    a = 1 / (T * tau);
    v = 1 / T;
    if t < 0
        s = 0;
    elseif t <= tau
        s = a * t^2 /2;
    elseif t> tau && t <= T
        s_tau = a * tau^2 /2;
        s = s_tau + v * (t-tau);
    elseif t>T && t<= (T+tau)
        s_tau = a * tau^2 /2;
        s_T = s_tau + v * (T-tau);
        s = s_T + v * (t-T) - a * (t-T)^2 / 2;
    elseif t> (T+tau)
        s = 1;
    end
end
