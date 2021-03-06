function [f] = movil_optimo_2(z,Q,R,hd,h,q,l,ts,N,k,vreal,x,obj,bandera)

%a) Acciones de control
vc = z ; %z=[u,w,q1_p,q2_p,q3_p,q4_p]';
VC=[z(1,:)';z(2,:)';z(3,:)';z(4,:)'];

%b) Estados de control
th1 = q(2);
th2 = q(3);

%c) Paramtros del Manipulador Movil
a = l(1);

%generacion del vector vacio para los errores de control
L= zeros(N*4,1);
L(1:4,1)=Fun(hd(:,1),h(:,1));

%% Norma para definir la distancia entre los objetos
%% definicion de los parametros para  la distancia del obejto
r=0.2;
b=-0.14;
D =zeros(N,1);
D(1,1)=evasion(h(:,1),r,b);

%% generacion de las acciones de control vacias
u1= zeros(1,N);
w1= zeros(1,N);

%% Accion de la accion de control en el interado inicial del sistema
u1(1)=vreal(1);
w1(1)=vreal(2);

%% generacion de las acciones de control vacias
u2= zeros(1,N);
w2= zeros(1,N);

%% Accion de la accion de control en el interado inicial del sistema
u2(1)=vreal(3);
w2(1)=vreal(4);

%% Variable auxiliar para poder almacenar el vector total de los errores del sistema
control_size = size(z);

%% auxiliar
aux1=control_size(1)+1;

for i=1:N-1
    
    %% GENERACION DE LA DINAMICA DEL ROBOT UNO
    v_real1=[u1(i),w1(i)]';
    
    estados1=[0 th1(i)]';
    
    vref1=vc(1:2,i);
    
    movil1 = MOVIL_DINAMICA(vref1,v_real1,estados1,ts,x);
    
    %% GENERACION DE LA DINAMICA DEL ROBOT DOS
    v_real2=[u2(i),w2(i)]';
    
    estados2=[0 th2(i)]';
    
    vref2=vc(3:4,i);
    
    movil2 = MOVIL_DINAMICA(vref2,v_real2,estados2,ts,x);
    
    %% VELOCIDADES DEL ROBOT 1
    u1(i+1)=movil1(1);
    w1(i+1)=movil1(2);
    
    %2) Matriz Jacobiana 1
    j11 = cos(th1(i));
    j12 = -a*sin(th1(i));
    
    
    j21 = sin(th1(i));
    j22 = +a*cos(th1(i));
    
    %% Definnicion de la matriz general ddel jacobiano 1
    J = [j11 j12;
        j21 j22];
    
    
    %% Seccion para saber los estados del sistema 1
    v1=[u1(i+1);w1(i+1)];
    
    th1(i+1) = th1(i)+v1(2)*ts;
    
    %% VELOCIDADES DEL ROBOT 2
    u2(i+1)=movil2(1);
    w2(i+1)=movil2(2);
    
    %2) Matriz Jacobiana
    P11 = cos(th2(i));
    P12 = -a*sin(th2(i));
    
    
    P21 = sin(th2(i));
    P22 = +a*cos(th2(i));
    
    %% Definnicion de la matriz general ddel jacobiano
    P = [P11 P12;
        P21 P22];
    
    
    %% Seccion para saber los estados del sistema
    v2=[u2(i+1);w2(i+1)];
    
    v= [u1(i+1);w1(i+1);u2(i+1);w2(i+1)];
    
    T=[J,zeros(2,2);
       zeros(2,2),P];
   
    h(:,i+1)=ts*T*v+h(:,i);
    
    th2(i+1) = th2(i)+v2(2)*ts;
    
    %% Generacion del vector de los errores del sistema
    L(aux1:(aux1+3),1)=(Fun(hd(:,k+i),h(:,i+1)));
    
    aux1=aux1+control_size(1);
    
    D(i,1)=evasion(h(:,i+1),r,b);
    
    
end
%% seccion para Seleccionar si se quiere o no evasion de obstaculos
Q = Q*eye(length(L));
R = R*eye(length(VC));
W = 0.009*eye(length(D));

%% Funcion costo a minizar tomando en cuanta las acciones de control del sistema
f=L'*Q*L+VC'*R*VC+2*(D'*W*D);

end
function [F] = Fun(hd,h)
% Generacion de los os errores del sistema
he = hd-h;
F=he;
end

function [obstaculo]=evasion(h,r,b)
beta=1;
d=norm(h(1:2,1)-h(3:4,1),2);

Aux =r-d;

obstaculo=beta*(1/(1+exp(-70*(Aux+b))));

end
