function [q, dq, r, rd, rdd, tmax] = jotraj(qd, R, qv, qa, dt)
%% Funcion para generar trayectorias entre puntos articulares
% qd = vector de posiciones articulares
% qv = maxima velocidad articular por trayecto
% qa = maxima aceleracion articular por trayecto
%%
n = length(qd(:,1))-1;

if nargin < 3
    % velicidades articulares maximas
    dqlim = [deg2rad(85) deg2rad(85) deg2rad(100) deg2rad(75) deg2rad(130) deg2rad(135) deg2rad(135)];
    ddqlim = dqlim / 0.8; % Se asume que el robot llegara en 0.5 s a su velocidad maxima linealmente -> a = dqlim/0.5
    for i=1:n
        qv(i) = min(dqlim);
        qa(i) = min(ddqlim);
    end
    dt = 0.05;
elseif nargin < 5
    dt = 0.05;
end


T = zeros(n,1);
tau = T;

for j = 1:n
    dq = max(abs(qd(j+1,:) - qd(j,:))); %dq = desplazamiento total    
    [T(j),tau(j)] = tlpar(dq, qv(i), qa(i), dt);
end

tmax = sum(T)+sum(tau);
m = round(tmax / dt) + 1;

q = zeros(m,7);
dq = q;
ddq = q;

r = zeros(m,6);
l = 1;


for k= 1: n
    
    qstart = qd(k,:);
    qtarget = qd(k+1,:);
    
    for t = 0 : dt : T(k)+tau(k)
        
        if l ~= 1 && t == 0
            continue;
        end
        
        s = sfun(t, T(k), tau(k));
        ds = sdfun(t, T(k), tau(k));
        dds = safun(t, T(k), tau(k));
        
        q(l,:) = qstart + s * (qtarget - qstart);
        dq(l,:) = ds * (qtarget - qstart);
        ddq(l,:) = dds * (qtarget - qstart);
        
        T_ = R.fkine(q(l,:));
        r(l,:) = [T_(1:3,4)', tr2rpy(T_(1:3,1:3))];
        l = l + 1;
    end
end

rd = zeros(size(r));

for i = 1 : length(q)
    rd(i,:) = R.jacob0(q(i,:)) * dq(i,:)';
end

rdd = diff(rd)/dt;

rdd = [[0 0 0 0 0 0];rdd];
end
