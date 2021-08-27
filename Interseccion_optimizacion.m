%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXCONTROL DE TRAYECTORIA DE UNA PLATAFORMA MOVILXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%% PARAMETROS DE TIEMPO
clc,close all,clear all;
ts=0.1;
tfinal=15;
to=0;
t=[to:ts:tfinal+ts];
load('DINAMICA_PLATAFORMA.mat');

%% DISTANCIAS DE LA PALTAFORMA MOVIL
a=0.15;
l=[a]';

%% POSICIONES INICIALES 1
hx1(1)=1;
hy1(1)=0;
phi1(1)=0*pi/180;

%% VELOCIDADES INICIALES 1
u1(1) = 0;
w1(1) = 0;

%% POSICIONES INICIALES 2
hx2(1)=0;
hy2(1)=1;
phi2(1)=0*pi/180;

%% VELOCIDADES INICIALES 2
u2(1) = 0;
w2(1) = 0;

%% CINEMATICA DIRECTA 1
hx1(1)=hx1(1)+a*cos(phi1(1));
hy1(1)=hy1(1)+a*sin(phi1(1));
hxp1(1)=0;
hyp1(1)=0;

%% CINEMATICA DIRECTA 2
hx2(1)=hx2(1)+a*cos(phi2(1));
hy2(1)=hy2(1)+a*sin(phi2(1));
hxp2(1)=0;
hyp2(1)=0;

%% Generacion de la trayectoeia deseada 1
hxd1=1*ones(1,length(t));
hyd1=0.1*t;
hd1=[hxd1;hyd1];

%% Generacion de la trayectoeia deseada 2
hxd2=0.1*t ;
hyd2=1*ones(1,length(t));
hd2=[hxd2;hyd2];
hd = [hxd1;hyd1;hxd2;hyd2];

%% Este valor cambiar si desea cambiar el horizonte de predciccion si se aumenta se demora mas optimizando
N=7;

%% RESTRICCION PARA LAS ACCIONES DE CONTROL
lb = [-0.5,-2.5,-0.5,-2.5]';
ub = [ 0.5, 2.5, 0.5, 2.5]';
LB=[];
UB =[];

for index=1:N-1
    LB=[LB;lb];
    UB=[UB;ub];
end

%% CONFIGURACION DEL METODO DE OPTIMIZACION A UTILIZAR
solver='fmincon';
b='Display';
c='off';
d='Algorithm';
e='sqp';
options=optimoptions(solver,b,c,d,e);

%% generacion de la solucion iniciale del sistema
z0=[u1(1)*ones(1,N-1);...
    w1(1)*ones(1,N-1);...
    u2(1)*ones(1,N-1);...
    w2(1)*ones(1,N-1)];


%% Generacion de los objetos a evadir
Obj=[hxd1(1,1:40:length(t));hyd1(1,1:40:length(t))];


%% Valores para matriz de ganancia
Q=1;
R=0.00001; %% Si se aumenta esa las acciones de control son mas suaves para puede que no se llega a erroes de cero

%% bandera para objetos
bandera=0;  %% 0 sin objetos 1 con objetos
for k=1:length(t)-N
    tic;
    
    hxe1(k)=hxd1(k)-hx1(k);
    hye1(k)=hyd1(k)-hy1(k);
    
    %% VECTORES PARA FORMA GENERAL PAA LOS DOS SISTEMAS
    h =[hx1(k),hy1(k),hx2(k),hy2(k)]';
    q = [0 phi1(k) phi2(k)]';
    
    %% generacion de los vectores de estados internos de cada robot
    q1=[0 phi1(k)]';
    q2=[0 phi2(k)]';
    
    %% GENERACION DE LOS VECTORES DE LAS VELOCIDADES REALES DEL SISTEMA
    v_real1=[u1(k) w1(k)]';
    v_real2=[u2(k) w2(k)]';
    v_real= [u1(k) w1(k) u2(k) w2(k)]';
    %% CONTROLADOR BASADO EN OPTIMIZACION
    [qref] = MPC(z0,Q,R,hd,h,q,l,ts,N,k,v_real,x,Obj,bandera,LB,UB,options);
    
    %% VELOCIDADES CINEMATICAS O VELOCIDADES DESEADAS PARA EL BLOQUE DE COMPENSACI�N DIN�MICA
    uref_c1(k)=qref(1,1);
    wref_c1(k)=qref(2,1);
    
    uref_c2(k)=qref(3,1);
    wref_c2(k)=qref(4,1);
    
    %% ERRORES DE VELOCIDAD VREF
    vref1 =[uref_c1(k) wref_c1(k)]';
    vref2 =[uref_c2(k) wref_c2(k)]';
    
    movil1 = MOVIL_DINAMICA(vref1,v_real1,q1,ts,x);
    
    movil2 = MOVIL_DINAMICA(vref2,v_real2,q2,ts,x);
    
    %% VELOCIDADES DEL ROBOT
    u1(k+1)=movil1(1);
    w1(k+1)=movil1(2);
    
    u2(k+1)=movil2(1);
    w2(k+1)=movil2(2);
    
    ue1(k)=uref_c1(k)-u1(k);
    we1(k)=wref_c1(k)-w1(k);
    
    
    %% POSICIONES DEL ROBOT
    phi1(k+1)=movil1(3);
    
    %% CINEMATICA DIRECTA
    hxp1(k+1)=u1(k+1)*cos(phi1(k+1))-a*w1(k+1)*sin(phi1(k+1));
    hyp1(k+1)=u1(k+1)*sin(phi1(k+1))+a*w1(k+1)*cos(phi1(k+1));
    
    hx1(k+1)=ts*hxp1(k+1)+hx1(k);
    hy1(k+1)=ts*hyp1(k+1)+hy1(k);
    
    %% POSICIONES DEL ROBOT
    phi2(k+1)=movil2(3);
    
    %% CINEMATICA DIRECTA
    hxp2(k+1)=u2(k+1)*cos(phi2(k+1))-a*w2(k+1)*sin(phi2(k+1));
    hyp2(k+1)=u2(k+1)*sin(phi2(k+1))+a*w2(k+1)*cos(phi2(k+1));
    
    hx2(k+1)=ts*hxp2(k+1)+hx2(k);
    hy2(k+1)=ts*hyp2(k+1)+hy2(k);
    
    z0 = [qref(1,:);...
          qref(2,:);...
          qref(3,:);...
          qref(4,:)];
    
    t_sample(k)=toc;
    
end
largo=0.4;
ancho=0.3;
close all, paso=1; 
paso=1; 
%a) Parámetros del cuadro de animación
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
h = light;
h.Color=[0.65,0.65,0.65];
h.Style = 'infinite';


    plot(hx1(1),hy1(1),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   
    plot(hxd1(1),hyd1(1),'Color',[32,185,29]/255,'linewidth',1.5);


for k = 1:1:length(t)-N
    drawnow
    
    
    plot(hxd1(1:k),hyd1(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    plot(hx1(1:k),hy1(1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    plot(hxd2(1:k),hyd2(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    plot(hx2(1:k),hy2(1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end
print -dpng SIMULATION_1
print -depsc SIMULATION_1

% SIMULACION(a,largo,ancho,hx,hy,hxd,hyd,phi,ts);
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

plot(hxd1(1,1:length(hx1)),hyd1(1,1:length(hx1)),'c'); hold on
plot(hxd2(1,1:length(hx1)),hyd2(1,1:length(hx1)),'c'); hold on
plot(hx1,hy1,'k'); grid
plot(hx2,hy2,'b'); grid
plot(Obj(1,:),Obj(2,:),'*r'); grid on
title('$\textrm{Trayectoria Descrita y Trayectoria Deseada}$','Interpreter','latex','FontSize',13);
legend({'$\mathbf{\eta}_{p_{des1}}$','$\mathbf{\eta}_{p_{des2}}$','$\mathbf{\eta}_{p1}$','$\mathbf{\eta}_{p2}$'},'Interpreter','latex','FontSize',13);
xlabel('$x[m]$','Interpreter','latex','FontSize',13); ylabel('$y[m]$','Interpreter','latex','FontSize',13);
print -dpng Sistema_1
print -depsc Sistema_1

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)

plot(t(1:length(hxe1)),hxe1,'r'); hold on
plot(t(1:length(hxe1)),hye1,'g'); hold on
grid on;
legend({'$\tilde{x_p}$','$\tilde{y_p}$'},'Interpreter','latex','FontSize',13);
title('$\textrm{Errores de Posicion}$','Interpreter','latex','FontSize',13);
xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',13); ylabel('$\textrm{Error}[m]$','Interpreter','latex','FontSize',13);

subplot(2,1,2)
plot(t(1,1:length(hxe1)),ue1,'k'); hold on
plot(t(1,1:length(hxe1)),we1,'c'); hold on
grid on;
legend({'$\tilde\mu$','$\tilde{\dot\psi_{p}}$'},'Interpreter','latex','FontSize',13);
title('$\textrm{Errores de Velocidad}$','Interpreter','latex','FontSize',13);
xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',13); ylabel('$\textrm{Error}[m/s][rad/s]$','Interpreter','latex','FontSize',13);
print -dpng ERROR_SISTEMA
print -depsc ERROR_SISTEMA

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t(1:length(u1)),u1,'r'); hold on
plot(t(1:length(u1)),w1,'c'); hold on
plot(t(1:length(uref_c1)),uref_c1,'--r'); hold on
plot(t(1:length(uref_c1)),wref_c1,'--c'); hold on
%     plot(t(1:length(uref_f)),uref_f,'--k'); hold on
%     plot(t(1:length(uref_c)),vrefp_w,'--m'); hold on
grid on
title('$\textrm{Velocidades a la Salida del Robot y Velocidades de Control Cinematico}$','Interpreter','latex','FontSize',13);
legend({'$\mu$','$\dot\psi_{p}$','$\mu_{ref_{c}}$','$\dot\psi_{p_{ref_{c}}}$'},'Interpreter','latex','FontSize',13);
xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',13); ylabel('$\textrm{Velocidad}[m/s][rad/s]$','Interpreter','latex','FontSize',13);
print -dpng CONTROL_VALUES
print -depsc CONTROL_VALUES

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(t_sample)),t_sample,'r'); hold on

grid on
title('$\textrm{Sample Time}$','Interpreter','latex','FontSize',13);
legend({'$t_{sample}$'},'Interpreter','latex','FontSize',13);
xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',13); ylabel('$\textrm{Velocidad}[m/s][rad/s]$','Interpreter','latex','FontSize',13);
print -dpng sample
print -depsc sample

emsx =hxe1*hxe1';
emsy =hye1*hye1';
