function [q_list] = reconfiguracion(q_pose, Robot, T07)

    delta_theta = deg2rad(2);
    
    q3_1 = q_pose(3) + delta_theta;
    q3_2 = q_pose(3) - delta_theta;
    
    q_1_next = alikeness(q_pose, inverse_kinematics(q3_1, T07, Robot));
    q_2_next = alikeness(q_pose, inverse_kinematics(q3_2, T07, Robot));
    
    cond_q_pose = cond(Robot.jacob0(q_pose));
    cond_q_actual = cond_q_pose;
    cond_q_1 = cond(Robot.jacob0(q_1_next));
    cond_q_2 = cond(Robot.jacob0(q_2_next));
    
    if cond_q_1 < cond_q_pose
        sgn = +1;
        q_next = q_1_next;
        cond_q_next = cond_q_1;
    elseif cond_q_2 < cond_q_pose && cond_q_2 < cond_q_1
        sgn = -1;
        q_next = q_2_next;
        cond_q_next = cond_q_2;
    end
    
    if (cond_q_1 >= cond_q_pose && cond_q_2 >= cond_q_pose)
        q_list = q_pose;
        return
    end
    
    n = 2;
    q_list(:, 1) = q_pose;
    while (cond_q_next < cond_q_actual)
%      while(1)
        
        q_list(:, n) = q_next;
        n = n + 1;
        
        q_actual = q_next;
        cond_q_actual = cond_q_next;
        
        q_next = alikeness(q_actual, inverse_kinematics(q_actual(3) + ...
            sgn*delta_theta, T07, Robot));
        
        cond_q_next = cond(Robot.jacob0(q_next));
        
        if n > 250
            break
        end
        
    end
    
end
