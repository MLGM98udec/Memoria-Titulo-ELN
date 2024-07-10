function [Y, X] = Smiulacion_espacio_estados(A, B, C, D, U, x0)
   
    % A, B, C, D - matrices del sistema
    % U - matriz de entrada (cada columna es un vector de entrada en cada instante de tiempo)
    % x0 - estado inicial del sistema
    
    simTime = size(U, 2);  % Número de pasos de tiempo de la simulación
    n = size(A, 1);        % Dimensión del estado (número de filas de A)
    r = size(C, 1);        % Número de salidas (número de filas de C)
    
    % Inicializar las matrices de estados y salidas
    X = zeros(n, simTime+1);  % Matriz de estados (n x (simTime+1))
    Y = zeros(r, simTime);    % Matriz de salidas (r x simTime)
    
    % Simular el sistema en cada paso de tiempo
    for i = 1:simTime
        if i == 1
            X(:, i) = x0;            % Estado inicial
            Y(:, i) = C * x0 + D * U(:, i);  % Salida inicial con entrada directa
            X(:, i+1) = A * x0 + B * U(:, i);  % Actualizar el estado
        else
            Y(:, i) = C * X(:, i) + D * U(:, i);  % Calcular la salida con entrada directa
            X(:, i+1) = A * X(:, i) + B * U(:, i);  % Actualizar el estado
        end
    end
end