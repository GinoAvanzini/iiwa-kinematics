function valid = check_qlim(R, q)
% Chequea límites articulares de la configuración q
    valid = true;
    for i = 1:7
        if q(i) > R.qlim(i, 2) || q(i) < R.qlim(i, 1)
            valid = false;
            break;
        end
    end
end
