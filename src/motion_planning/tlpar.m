function [T,tau] = tlpar(dq, qv, qa, dt)
%%
%   Funcion para calcular los parametros de nuestra ley de tiempo. 
%   Se usan los parametros mas defavorables y despues las otras trayectorias se escalan 
%   dq = desplazamiento total
%   qv = velocidad maxima
%   qa = aceleracion maxima 
%   dt = tiempo del ciclo de control
%
%%
    if nargin < 4
       dt = 0.01;
    end
    
    T = dq / qv;
    tau = qv / qa;
    
    
    % Si tau es mayor a T --> elijo tau como una ley triangular y para
    % mantener el trapecio elijo T = qv/qa que es el valor que ya obtuve
    % anteriormente para mantener mi ley trapezoidal
    if T < tau
        T = tau;
        tau = sqrt(dq / qa);
    end
    
    % Redondeo de los parametros T y tau
    T = dt * ceil(T / dt);
    tau = dt * ceil(tau / dt);
    
    % Verifico T y tau no sean menores a Tmin y  taumin
    K = 4; % Factor que define Tmin y taumin admisible
    if T < K * dt,  T = K * dt; end
    if tau < K * dt, tau = K * dt; end
    
end
