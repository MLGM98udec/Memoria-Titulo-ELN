% Controlador MPC -2
%ecuaciones_de_estado_completo_x;
A= [            0  -0.00081454    -0.035336            0;
            0    -0.086973      -1.0254            0;
       9.1371       9.1371      -10.278            0;
            0            0            0      -7.3235];
B = [      0.47619            0      -4.8609            0;
       13.819   -0.0016082      -141.06            0;
            0            0      -201.21            0;
            0            0            0       7.3235];
C = eye(4);
D = zeros(4);



ts = 0.1;
[phi,gm] = c2dmp(A,B,ts); % Modelo Discreto
mod = ss2mod(phi,gm,C,D); % Espacio de estado en formato mod
delt =2;
%plotstep(mod2step(mod,150))
P = 20; % Horizonte de prediccion
M =1;
t = 100; % Tiempo de simulacion
imod = mod;
ywt = []; % Peso en las entradas
uwt = []; % Peso en las entradas
Ks = smpccon(imod,ywt,uwt,M,P);
Q = [1 0;0 1];
R = diag([1 1 1 1]);
r = [0.4 5 6 0.6]; % Set-Point
%kest = smpcest(imod,Q,R);

SP = zeros(4, t);
SP(1, :) = r(1) * (1:t >= 2); 
SP(2,:) =  r(2) * (1:t >= 2);   
SP(3,:) =  r(3)* (1:t >= 2); 
SP(4,:) =  r(4) *(1:t >= 2);



[y u]= smpcsim(mod,imod,Ks,t,r);



%figure(1);
for i = 1:4
    %subplot(2, 2, i); % Crear una cuadrícula de 2x2 para los gráficos
    figure(i);
    plot(y(:,i),'LineWidth', 3);
    hold on;
    plot(SP(i, :),'LineWidth', 3)
    hold off;
    if i == 1        
        xlabel('time steps');
        ylabel('metros');
        title('Altura (h)');
        legend('h', 'Setpoint')

        

    end
    if i == 2        
        xlabel('time steps');
        ylabel('mh20');
        title('Presion (P)');
        legend('P', 'Setpoint')
    end
    if i == 3
        xlabel('time steps');
        ylabel('metros');
        title('Cambio de Fibra en SL (hj)');
        legend('hj', 'Setpoint')
    end
    if i == 4
        xlabel('time steps');
        ylabel('%');
        title('Consistencia de SL (Cs)');
        legend('Cs', 'Setpoint')
    end
end


