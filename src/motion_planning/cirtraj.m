function [q, qdot, r, rd, rdd, tmax] = cirtraj(PSt ,T0C, R, qant, ang, qv, qa, dt)
%% Funcion para generar circulos en el espacio
% T0C = T0C transformacion homogenea desde la base al centro del circulo
% PSt = P inicial del circulo
%
%%

if nargin < 5
    qv = 1;
    qa = 0.5;
    dt = 0.05;
    ang = 2 * pi;
elseif nargin < 6
    qv = 1;
    qa = 0.5;
    dt = 0.05;
elseif nargin <8
    dt = 0.05;
    
end

ro = sqrt(sum((PSt(1:3,4) - T0C(1:3,4)).^2));

dq = ang * ro;

[T,tau] = tlpar(dq, qv, qa , dt);

tmax = T+tau;
t = 0 : dt : tmax;

n = length(t);

p = zeros(n,3);
dp = p;
ddp = p;

fi = p;
dfi = p;
ddfi = p;

T_ = zeros(4,4,n);
q = zeros(n,7);

fistart = tr2rpy(T0C(1:3,1:3));
fitarget = fistart;

TCP = inv(T0C) * PSt;

tita0 = atan2(TCP(2,4), TCP(1,4));
tita1 = ang + tita0;

for l=1:n
    s = sfun(t(l), T, tau);
    ds = sdfun(t(l), T, tau);
    dds = safun(t(l), T, tau);
    
    arcs = tita0 * ro + s * (tita1 - tita0) * ro;
    
    ps = [ro * cos(arcs/ro); ro * sin(arcs/ro); 0 ];
    dps = [ -sin(arcs/ro); cos(arcs/ro); 0];
    ddps = [-cos(arcs/ro)/ro; -sin(arcs/ro)/ro; 0];
    
    p(l,:) = (T0C(1:3,4)+ T0C(1:3,1:3) * ps)';
    dp(l,:) = (ds * T0C(1:3,1:3) * dps)';
    ddp(l,:) = (T0C(1:3,1:3) * (dds * dps + ds^2 * ddps))';
    
    fi(l,:) = fistart + s * (fitarget - fistart);
    dfi(l,:) = ds * (fitarget - fistart);
    ddfi(l,:) = dds * (fitarget - fistart);
    
    T_(:,:,l) = transl(p(l,:)) * rpy2tr(fi(l,:));
    
    q(l,:) = pinoikine(qant',T_(:,:,l),R);
    qant = q(l,:);
    
end

r = [p fi];
rd = [dp dfi];
rdd = [ddp ddfi];

qdot = zeros(size(q));

for i = 1 : length(q)
    qdot(i,:) = pinv(R.jacob0(q(i,:))) * rd(i,:)';
end
end