function [q, qdot, r, rd, rdd, tmax] = cirtraj2(PSt ,PTt, R, qv, qa, dt)
%% Funcion para generar circulos entre 2 puntos 
% PTt = P target del circulo 
% PSt = P inicial del circulo 
% qv = maxima velocidad carteciana 
% qa = maxima aceleracion carteciana  
%%
    if nargin < 4
        qv = 1;
        qa = 0.5;
        dt = 0.05;
    elseif nargin <6
        dt = 0.05;
    end

    % velicidades articulares maximas
    dqlim = [deg2rad(85) deg2rad(85) deg2rad(100) deg2rad(75) deg2rad(130) deg2rad(135) deg2rad(135)];
    ddqlim = dqlim / 0.8; % Se asume que el robot llegara en 0.5 s a su velocidad maxima linealmente -> a = dqlim/0.5


    fistart = tr2rpy(PSt.R, 'xyz');
    fitarget = tr2rpy(PTt.R, 'xyz');

    ro = sqrt(sum((PTt.t - PSt.t).^2)) / 2;

    qps = R.ikine(PSt);
    qpt = R.ikine(PTt);

    difqp = abs(qpt - qps);
    diffi = sqrt(sum((fitarget - fistart).^2));

    difcp = pi * ro;

    dq = horzcat(difcp, diffi, difqp);
    qv = horzcat(qv, qv, dqlim);
    qa = horzcat(qa, qa, ddqlim);

    imax = length(dq);

    T = zeros(1,imax);
    tau = zeros(1,imax);

    for i= 1 : imax
        [T(i),tau(i)] = tlpar(dq(i), qv(i), qa(i) , dt);
    end

    % Encuentro mi perfil mas exigente
    T = max(T);
    tau = max(tau);

    tmax = T+tau+dt;
    t = 0 : dt : tmax;

    n = length(t);

    p = zeros(n,3);
    dp = zeros(n,3);
    ddp = zeros(n,3);

    fi = zeros(n,3);
    dfi = zeros(n,3);
    ddfi = zeros(n,3);

    T_ = zeros(4,4,n);
    q = zeros(n,7);

    qant = [0 0 0 0 0 0 0];

    T0C = PSt.inv *  PTt;
    Taux = T0C.T;

    Taux(1:3,4) = Taux(1:3,4) / 2 + PSt.t;
    T0C = SE3.check(Taux);

    TCPS = T0C.inv * PSt;

    tita0 = atan2(TCPS.t(2), TCPS.t(1));
    tita1 = pi + tita0;

    for l=1:n
        s = sfun(t(l), T, tau);
        ds = sdfun(t(l), T, tau);
        dds = safun(t(l), T, tau);

        arcs = tita0 * ro + s * (tita1 - tita0) * ro;

        ps = [ro * cos(arcs/ro); ro * sin(arcs/ro); 0 ];
        dps = [ -sin(arcs/ro); cos(arcs/ro); 0];
        ddps = [-cos(arcs/ro)/ro; -sin(arcs/ro)/ro; 0];

        p(l,:) = (T0C.t + T0C.R * ps)';
        dp(l,:) = (T0C.R * dps * ds)';
        ddp(l,:) = (T0C.R * (dps * dds + ddps * ds^2))';

        fi(l,:) = fistart + s * (fitarget - fistart);
        dfi(l,:) = ds * (fitarget - fistart);
        ddfi(l,:) = dds * (fitarget - fistart);

        T_(:,:,l) = transl(p(l,:)) * rpy2tr(fi(l,:),'xyz');

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
