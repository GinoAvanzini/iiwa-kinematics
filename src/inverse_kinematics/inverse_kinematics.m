function [q] = inverse_kinematics(q3_in, T07, Robot)
% Cálculo de cinemática inversa
% Recibe la transformación objetivo (posición y orientación) y el 
% q_3 con el que se quiere obtener la solución.
% Devuelve las 8 soluciones

    q = zeros(7, 8);
    DH = [Robot.theta', Robot.d', Robot.a', Robot.alpha'];
    
    % La transformación objetivo (que está con respecto al origen
    % global) se transforma al origen en la base del robot.
    T0_7 = inv(Robot.base)*T07;     

    P0_7 = T0_7(1:3, 4);

    P0_W = real(P0_7 - DH(7, 2)*T0_7(1:3,3));     % Wrist

    % Ubicación de la articulación 1 - Independiente de qi
    T0_1 = Robot.links(1).A(0);
    P0_1 = T0_1(1:3, 4);

    r1_4 = real(norm(P0_1 - P0_W));

    q4(1) = real(pi - acos( (DH(3, 2)^2 + DH(5, 2)^2 - r1_4^2) / ...
                    (2*DH(3, 2)*DH(5, 2)) ));

    q4(2) = real(pi + acos( (DH(3, 2)^2 + DH(5, 2)^2 - r1_4^2) / ...
                    (2*DH(3, 2)*DH(5, 2)) ));

    q3 = q3_in;

    q(3, :) = q3;
    q(4, 1:2) = [q4(1), q4(1)];
    q(4, 3:4) = [q4(2), q4(2)];
    q(4, 5:8) = q(4, 1:4);

    d1 = DH(1, 2);
    xw = P0_W(1);
    yw = P0_W(2);
    zw = P0_W(3);

    n = 1;

    for u=1:2       % q4

        T25 = Robot.A([3 4 5], [0 0 q3 q4(u) 0 0 0]);

        a = real(T25(3, 4));
        b = real(-T25(1, 4));
        R_ab = (a^2 + b^2)^0.5;
        alpha_conv = real(atan2(b, a));

        q2(1) = real(alpha_conv - acos((zw - d1)/R_ab));
        q2(2) = real(alpha_conv + acos((zw - d1)/R_ab));

        for v=1:2       % q2
            T15 = Robot.links(2).A(q2(v))*T25;     % T15 = T12*T25

            a = real(T15(1, 4));
            b = real(-T15(3, 4));
            R_ab = (a^2 + b^2)^0.5;
            alpha_conv = real(atan2(b, a));

            q1(1) = real(alpha_conv - acos(xw/R_ab));
            q1(2) = real(alpha_conv + acos(xw/R_ab));

            T04_actual_q1_1 = Robot.A([1 2 3 4 5], [q1(1), q2(v), q3, q4(u), 0, 0, 0]);
            T04_actual_q1_2 = Robot.A([1 2 3 4 5], [q1(2), q2(v), q3, q4(u), 0, 0, 0]);

            d_q1 = abs(norm(T04_actual_q1_1(1:3, 4) - [xw, yw, zw]'));
            d_q2 = abs(norm(T04_actual_q1_2(1:3, 4) - [xw, yw, zw]'));

            if (d_q1 <= d_q2)
                q1 = q1(1);
            else
                q1 = q1(2);
            end

            q(1, n) = q1;
            q(2, n) = q2(v);
            q(1, n + 4) = q1;       % Solución igual para segunda muñeca
            q(2, n + 4) = q2(v);

            % Muñeca
            T0_4 = Robot.A([1 2 3 4], ...
                [q(1, n), q(2, n), q(3, n), q(4, n), 0 0 0]);
            T4_7 = inv(T0_4) * T0_7;

            q6 = -atan2(sqrt(T4_7(3,1)^2 + T4_7(3,2)^2), T4_7(3,3));
            q7 = atan2(-T4_7(3,2), T4_7(3,1));
            q5 = atan2(T4_7(2,3), T4_7(1,3)) + pi;
            q(5, n) = q5;
            q(6, n) = q6;
            q(7, n) = q7;

            q6 = atan2(sqrt(T4_7(3,1)^2 + T4_7(3,2)^2), T4_7(3,3));
            q7 = atan2(-T4_7(3,2), T4_7(3,1)) - pi;
            q5 = atan2(T4_7(2,3), T4_7(1,3));
            q(5, n + 4) = q5;
            q(6, n + 4) = q6;
            q(7, n + 4) = q7;

            n = n + 1;
        end
    end

    for u = 1:7     % Normalización de ángulos a [-pi, pi]
        for v = 1:8
            if q(u, v) > pi
                q(u, v) = q(u, v) - 2*pi;
            elseif q(u, v) < -pi
                q(u, v) = q(u, v) + 2*pi;
            end
        end
    end

end
