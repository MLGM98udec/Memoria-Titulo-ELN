classdef ControlPredictivoModel
    properties
        A
        B
        C
        f
        v
        W3
        W4
        n
        r
        m
        SetPoint
        tiempoActual = 0
        estados
        inputs
        outputs
        O
        M
        K
    end
    
    methods
        function obj = ControlPredictivoModel(A, B, C, f, v, W3, W4, x0, SetPoint)
            obj.A = A;
            obj.B = B;
            obj.C = C;
            obj.f = f;
            obj.v = v;
            obj.W3 = W3;
            obj.W4 = W4;
            obj.SetPoint = SetPoint;
            
            obj.n = size(A, 1);
            obj.r = size(C, 1);
            obj.m = size(B, 2);
            
            obj.estados = {x0};
            obj.inputs = {};
            obj.outputs = {};
            
            [obj.O, obj.M, obj.K] = obj.formarMatricesElevadas();
        end
        
        function [O, M, K] = formarMatricesElevadas(obj)
            f = obj.f;
            v = obj.v;
            r = obj.r;
            n = obj.n;
            m = obj.m;
            A = obj.A;
            B = obj.B;
            C = obj.C;

            O = zeros(f * r, n);
            for i = 1:f
                if i == 1
                    powA = A;
                else
                    powA = powA * A;
                end
                O((i - 1) * r + 1:i * r, :) = C * powA;
            end

            M = zeros(f * r, v * m);
            for i = 1:f
                if i <= v
                    for j = 1:i
                        if j == 1
                            powA = eye(n);
                        else
                            powA = powA * A;
                        end
                        M((i - 1) * r + 1:i * r, (i - j) * m + 1:(i - j + 1) * m) = C * powA * B;
                    end
                else
                    for j = 1:v
                        if j == 1
                            powA = eye(n);
                        else
                            powA = powA * A;
                        end
                        M((i - 1) * r + 1:i * r, (v - j) * m + 1:(v - j + 1) * m) = C * powA * B;
                    end
                end
            end

            % Corrección en el cálculo de K
            K = (M' * obj.W4 * M + obj.W3) \ (M' * obj.W4');
        end
        
        function obj = ControlInputs(obj)
            obj.tiempoActual = obj.tiempoActual + 1;
            controlDeseadoActual = obj.SetPoint(obj.tiempoActual:obj.tiempoActual + obj.f - 1, :)';
            trayectoriaControlDeseado = controlDeseadoActual(:);
            if obj.tiempoActual == 1
                estadoActual = obj.estados{obj.tiempoActual};
            else 
                estadoActual = obj.outputs;

            end 
            estadoActual = obj.estados{obj.tiempoActual};
            
            % % Verificar las dimensiones de obj.O y estadoActual
            % if size(obj.O, 2) ~= length(estadoActual)
            %     error('Dimensiones incompatibles entre obj.O y el estado actual estadoActual.');
            % end
            
            vectorS = trayectoriaControlDeseado - obj.O * estadoActual;
            controlCalculado = obj.K * vectorS;
            
            % Aplicar la primera entrada de control calculada
            obj.inputs{obj.tiempoActual} = controlCalculado(1:obj.m);
            
            % Calcular el nuevo estado y salida del sistema
            nuevoEstado = obj.A * estadoActual + obj.B * obj.inputs{obj.tiempoActual};
            nuevaSalida = obj.C * estadoActual;
            
            obj.estados{obj.tiempoActual + 1} = nuevoEstado;
            obj.outputs{obj.tiempoActual} = nuevaSalida;
        end
    end
end