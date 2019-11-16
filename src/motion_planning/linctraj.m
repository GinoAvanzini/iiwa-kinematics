function [q, qdot, r, rd, rdd, tmax] = linctraj(pd, fid, R, qant, qv, qa, dt)
%% Funcion para generar trayectorias lineales entre puntos cartesianos
% pd = matriz Ix3 con las posiciones deseadas
% fid = matriz Ix3 con las orientaciones
% qv = maxima velocidad cartesiana
% qa = maxima aceleracion cartesiana
%%
n = length(pd(:,1));

if nargin < 5
    for i=1:n
        qv(i) = 1;
        qa(i) = 0.5;
    end
    dt = 0.05;
elseif nargin < 7
    dt = 0.05;
end

T = zeros(n-1,1);
tau = zeros(n-1,1);

for j = 1:n-1
    dis_p = sqrt(sum((pd(j+1,:) - pd(j,:)).^2));
    dis_fi = sqrt(sum((fid(j+1,:) - fid(j,:)).^2));
    dq = max([dis_p dis_fi]); %dq = desplazamiento total
    [T(j),tau(j)] = tlpar(dq, qv(i), qa(i), dt);
end

tmax = sum(T) + tau(end);
T_ant = 0;
k = 1;
l = 1;
m = round(tmax/dt) + 1;
p = zeros(m,3);
dp = p;
ddp = p;
fi = p;
dfi = p;
ddfi = p;
T_ = zeros(4,4,m);
q = zeros(m,7);

for t = 0 : dt : tmax
        
    s = sfun(t - T_ant, T(k), tau(k));
    ds = sdfun(t - T_ant, T(k), tau(k));
    dds = safun(t - T_ant, T(k), tau(k));
    
    T2 = sum(T(1:k));
    s2 = sfun(t-T2, T(k+1), tau(k+1));
    ds2 = sdfun(t-T2, T(k+1), tau(k+1));
    dds2 = safun(t-T2, T(k+1), tau(k+1));
    
    p(l,:) = pd(k,:) + s * (pd(k+1,:) - pd(k,:)) + s2 * (pd(k+2,:)-pd(k+1,:));
    dp(l,:) = ds * (pd(k+1,:) - pd(k,:)) + ds2 * (pd(k+2,:)-pd(k+1,:));
    ddp(l,:) = dds * (pd(k+1,:) - pd(k,:)) + dds2 * (pd(k+2,:)-pd(k+1,:));
    
    fi(l,:) = fid(k,:) + s * (fid(k+1,:) - fid(k,:)) + s2 * (fid(k+2,:) - fid(k+1,:)) ;
    dfi(l,:) = ds * (fid(k+1,:) - fid(k,:)) + ds2 * (fid(k+2,:) - fid(k+1,:));
    ddfi(l,:) = dds * (fid(k+1,:) - fid(k,:)) + dds2 * (fid(k+2,:) - fid(k+1,:));
    
    T_(:,:,l) = transl(p(l,:)) * rpy2tr(fi(l,:));
    q(l,:) = pinoikine(qant',T_(:,:,l),R);
    qant = q(l,:);
    
    if (s==1) && (k ~= n-2)
        T_ant = T2;
        k = k+1;
    end
    l = l + 1;
end

r = [p fi];
rd = [dp dfi];
rdd = [ddp ddfi];

qdot = zeros(size(q));

for i = 1 : length(q)
    qdot(i,:) = pinv(R.jacob0(q(i,:))) * rd(i,:)';
end
end