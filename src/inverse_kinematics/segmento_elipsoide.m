function l = segmento_elipsoide(Jacob, veloc)
% Devuelve las tres componentes del vector en la direccion de vel 
% para el jacobiano dado

    J = Jacob(1:3, :);
    
    veloc = veloc/norm(veloc);
    
    ellipse_matrix = J*J';
    
    % La ecuación de la elipse es
    % X'*(J*J')^-1*X = 1
    ellipse_matrix = inv(ellipse_matrix);

    a = ellipse_matrix(1, 1);
    b = ellipse_matrix(2, 2);
    c = ellipse_matrix(3, 3);
    d = ellipse_matrix(1, 2);
    e = ellipse_matrix(1, 3);
    f = ellipse_matrix(2, 3);
    
    t = sqrt(1/(a*veloc(1)^2 + b*veloc(2)^2 + c*veloc(3)^3 + 2*d*veloc(1)*veloc(2) + ...
    2*e*veloc(1)*veloc(3) + 2*f*veloc(2)*veloc(3)));

    l = t*veloc;

end

