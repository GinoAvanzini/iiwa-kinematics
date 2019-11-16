function [q_alike] = alikeness(q_init, q_sol)
% Recibe un q_init columna y un q_sol(7x8)
% Devuelve el q_sol(:, i) más parecido a q_init

    q_alike = q_sol(:, 1);
    
    for i = 2:8
        if (sum(sum(abs(q_init - q_alike)))) > ...
                (sum(sum(abs(q_init - q_sol(:, i)))))
            q_alike = q_sol(:, i);
        end
    end
end

