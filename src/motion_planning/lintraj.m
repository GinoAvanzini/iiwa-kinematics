function [q, qdot, r, rd, rdd, tmax] = lintraj(pd, fid, R, qant, qv, qa, dt)
%% Funcion para generar trayectorias lineales entre puntos cartesianos
% pd = matriz Ix3 con las posiciones deseadas
% fid = matriz Ix3 con las orientaciones
% qv = maxima velocidad cartesiana
% qa = maxima aceleracion cartesiana
%%

n = length(pd(:,1))-1;

if nargin < 5
    for i=1:n
        qv(i) = 1;
        qa(i) = 0.5;
    end
    dt = 0.05;
elseif nargin < 7
    dt = 0.05;
end

% velicidades articulares maximas
%     dqlim = [deg2rad(85) deg2rad(85) deg2rad(100) deg2rad(75) deg2rad(130) deg2rad(135) deg2rad(135)];
%     ddqlim = dqlim / 0.8; % Se asume que el robot llegara en 0.5 s a su velocidad maxima linealmente -> a = dqlim/0.5

T = zeros(n,1);
tau = zeros(n,1);

for j = 1:n
    dis_p = sqrt(sum((pd(j+1,:) - pd(j,:)).^2));
    dis_fi = sqrt(sum((fid(j+1,:) - fid(j,:)).^2));
    dq = max([dis_p dis_fi]); %dq = desplazamiento total
    [T(j),tau(j)] = tlpar(dq, qv(i), qa(i), dt);
end


tmax = sum(T)+sum(tau);
m = round(tmax / dt) + 1;
p = zeros(m,3);
dp = p;
ddp = p;
fi = p;
dfi = p;
ddfi = p;
T_ = zeros(4,4,m);
q = zeros(m,7);
l = 1;

for k= 1: n
    
    pstart = pd(k,:);
    ptarget = pd(k+1,:);
    fistart = fid(k,:);
    fitarget = fid(k+1,:);
    for t = 0 : dt : T(k)+tau(k)
        
        if l ~= 1 && t == 0
            continue;
        end
        
        s = sfun(t, T(k), tau(k));
        ds = sdfun(t, T(k), tau(k));
        dds = safun(t, T(k), tau(k));
        
        p(l,:) = pstart + s * (ptarget - pstart);
        dp(l,:) = ds * (ptarget - pstart);
        ddp(l,:) = dds * (ptarget - pstart);
        
        fi(l,:) = fistart + s * (fitarget - fistart);
        dfi(l,:) = ds * (fitarget - fistart);
        ddfi(l,:) = dds * (fitarget - fistart);
        
        T_(:,:,l) = transl(p(l,:)) * rpy2tr(fi(l,:));
        
        q(l,:) = pinoikine(qant', T_(:,:,l), R);
        qant = q(l,:);
        l = l + 1;
    end
end

r = [p fi] ;
rd = [dp dfi];
rdd = [ddp ddfi];

qdot = zeros(size(q));

for i = 1 : length(q)
    qdot(i,:) = pinv(R.jacob0(q(i,:))) * rd(i,:)';
end
end
