function [q_next] = pinoikine(q_actual, T_next, Robot)
% Función de cinemática inversa para planificación de trayectorias
% Recibe la transformación objetivo del punto siguiente y la configuración anterior
% Devuelve una configuración articular optimizada para maximizar la 
% manipulabilidad en la dirección instantánea de movimiento Y para tener una
% configuración del tipo codo-arriba

    delta_q3 = 0.0013;
    delta_q3 = delta_q3/2;
    region = 0.026;
%     region = 0.013;
    region = 0.0065;
    region = region/2;
%     region = 0.052;
    
    aux = Robot.fkine(q_actual);
    vel = T_next(1:3, 4) - aux(1:3, 4);

    q_next = alikeness(q_actual, inverse_kinematics(q_actual(3), T_next, Robot));

    if isElbowUp(Robot, q_actual)   % then Optimizamos por manipulabilidad
        
        quality_manip = segmento_elipsoide(Robot.jacob0(q_next), vel);   % Maximize manipulability

        for i = (q_actual(3) - region) : delta_q3 : (q_actual(3) + region)

            q_temp = alikeness(q_actual, inverse_kinematics(i, T_next, Robot));
            quality_manip_temp = segmento_elipsoide(Robot.jacob0(q_temp), vel);

            if quality_manip_temp > quality_manip
                quality_manip = quality_manip_temp;
                q_next = q_temp;
            end

        end
        
        return
        
    else        % then optimizamos por altura en z
        
        T_act = Robot.A([1 2 3 4], q_next);
        quality_z = T_act(3, 4);      % maximize Z position

        for i = (q_actual(3) - region) : delta_q3 : (q_actual(3) + region)

            q_temp = alikeness(q_actual, inverse_kinematics(i, T_next, Robot));
            T_temp = Robot.A([1 2 3 4], q_temp);
            quality_z_temp = T_temp(3, 4);

            if quality_z_temp > quality_z
                quality_z = quality_z_temp;
                q_next = q_temp;
            end

        end

    end
    
end


function elbow_up = isElbowUp(Robot, q)

    P01 = Robot.A([1], q);
    P01 = P01(1:3, 4);
    
    P03 = Robot.A([1 2 3], q);
    P03 = P03(1:3, 4);
    
    P05 = Robot.A([1 2 3 4 5], q);
    P05 = P05(1:3, 4);
    
    P15 = P05 - P01;
    
    Pobj = P01 + P15*0.5;
    
    if P03(3) > (Pobj(3) + 0.05)
        elbow_up = true;
    else 
        elbow_up = false;
    end

end
