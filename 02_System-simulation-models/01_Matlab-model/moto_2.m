close all
clear all
clc
restoredefaultpath
addpath(genpath('D:\vehiculo-sim'));
%% 

%%%Carga los datos de los diferentes ciclos de manejo.
YUDDS0=load('UDDS.TXT','-ASCII');
YUDDS1=load('uddsextendido.txt','-ASCII');
YNYCC=load('NYCCCOL.TXT','-ASCII');
YFTP=load('FTP.txt','-ASCII');

%subrutina de calculo de ciclos
YUDDS=YUDDS0;%para asignar el primer ciclo
%% 

nUD=size(YUDDS);%para el valor final para la indexacion del siguiente ciclo
nCiclos=10;%numero de ciclos que se requieren<------------------------------------------
for i=1:nCiclos-1
    YUDDS(end+1:(end+nUD),:)=YUDDS0;
end
%%Realiza la correcta asignaci�n de variables y convierte unidades. 
tUDk=YUDDS(:,1);
%YUDDS=YUDDS1;%para comprobar
vUD=YUDDS(:,2);
vUDk=vUD*1.60934;
vUD=vUD*1.60934*(5/18);%
nUD=size(vUD);
tUD=0:nUD-1;%tiempo
xUD(1)=0;
%% 
%Par�metros del vehiculo
m=166 ; %kg
g=9.81 ; %m/s
alpha=0; %radianes
ro=1.2041; %kg/m3 Denisdad del medio
Af=0.72; %m2 Fuerza aerodinámica
Cd=0.84; %Coeficiente de arrastre (adimensional)
dt=1; %s Tiempo de muestreo del patr�n de manejo
rw=(15/2)*0.0254+.65*0.145; %radio de llanta 145/65 R15
%% 

%Par�metros del motor eléctrico
Js=0.01904; %Momento de inercia IEEE TPE Lian 2019
Bv=0.017; %Coeficiente de fricci�n viscosa
P=4; %Número de polos
%% 

%Par�metros del sistema de tracci�n 
G=1; %Depende del sistema de tracci�n. gear ratio
etaG=0.95; %Eficiencia del engrane
%% 

%Par�metros del inversor
etaInv=0.95; %Eficiencia del inversor
%Par�metros del convertidor
etaConv = 0.95;
%% 

%Parametros del ultracapacitor
UCser=0; %UC en serie
UCpar=0; %UC en paralelo
Vuc=3.0; %voltaje en volts del uc
Cuc=3400; %capacitancia del UC en F
ESRuc=0.15*10^-3; %resistencia del UC
ILuc=12*10^-3; %corriente de fuga del UC
Iucmax=225; %maxima corriente del UC a 40�C
Eucmax=4.25; %energia m�xima en el UC en Wh
Pesouc=0.496; %peso del UC en kg
Ructot=UCser*ESRuc/UCpar; %resistencia total del banco
Cuctot=UCpar*Cuc/UCser; %capacitancia del banco
Vuctot=Vuc*UCpar/UCser; %voltaje total del banco
%% 

%Parámetros de la bateria
RdisBat=0.55e-3; %ohms
RchaBat=0.55e-3;   %ohms
VinBat=3.7; %volts*numero de baterias en serie
nser= 15; %%bater�as en serie 
npar= 2; %%baterias en paralelo
DoD=0.0;
SOC (1)= 1 - DoD;
Qbat =85 ;%Ah
Enertotbat=Qbat*npar
QbatBanco = Qbat*npar*3600; %Qbat*npar/nser*3600
Pcellbat=Qbat*VinBat;
Ptotbat=Pcellbat*nser*npar
Ptotbatneg=-1*Ptotbat;
vBatUD(1)=VinBat;
vBatUD1(1)=VinBat*nser;
pesobat=1.735;

%% 

%Estado de descarga, constantes

a(10) = -634.0;
a(9) = 2942.1;
a(8) = -5790.6;
a(7) = 6297.4;
a(6) = -4132.1;
a(5) = 1677.7;
a(4) = -416.4;
a(3) = 60.5;
a(2) = -4.8;
a(1) = 0.2;
a(11) = 0.0;%%a0

b(10) = -8848;
b(9) = 40727;
b(8) = -79586; 
b(7) = 86018;
b(6) = -56135;
b(5) = -5565;
b(4) = 784;
b(3) = -25;
b(2) = 55;
b(1) = 0;
b(11) = 4;%%b0

c(10) = 2056;
c(9) = -9176;
c(8) = 17147;
c(7) = -17330;
c(6) = 10168;
c(5) = -3415;
c(4) = 578;
c(3) = 25;
c(2) = 3;
c(1) = 0; 
c(11) = 0;%%c0
%DoD=0.9;
%SOC (1)= 1 - DoD;
%% 

%C�lculo de la demanda de potencia
ftUD(1)=0;
ftNY(1)=0;
ftFTP(1)=0;

serie2=0;%solamente es una variable de apoyo
serie=0;

m = m+(npar*nser*pesobat)+(Pesouc*UCser*UCpar)  %kg con baterias
%% 
for i=2:nUD
    dist(1)=0;
   dist(i)=vUD(i)+dist(i-1); 
end
distancia=max(dist);
sprintf('largo del recorrido = %+5.2f km',distancia/1000)
%% 


for i=2:nUD
    Cr= 0.01*(1+ (0.036*vUD(i))); %resistencia al rodamiento
    fUD=m*(vUD(i)-vUD(i-1)/dt); %fuerza inercial
    fgUD=m*g*sin(alpha); %Fuerza gravitacional
    frUD= sign(vUD(i))*m*g*cos(alpha)*Cr; %fuerza de resistencia al rodamiento en las llantas
    faUD=sign(vUD(i))*0.5*ro*Cd*Af*vUD(i)*vUD(i); %fuerza del viento
    ftUD(i)= fUD + fgUD + frUD + faUD; %suma de fuerzas, fuerza necesaria para mover al vehiculo
    tauMUD(i)=ftUD(i)*rw; %torque de tracci�n
    PtUD(i)=ftUD(i)*vUD(i); %potencia de tracciÓn. NECESARIA PARA MOVER EL AUTO 
    WwUD(i)=vUD(i)/rw; % Velocidad angular de la llanta= VellinVe/Radllan. rad/s 
    if PtUD(i)>0
       PpUD(i)=PtUD(i);
       PnUD(i)=0;
       TauGUD(i)=tauMUD(i)/(etaG*G); %torque del motor electrico
        
    else
        PnUD(i)=PtUD(i);
        PpUD(i)=0;
        TauGUD(i)=etaG*tauMUD(i)/(G);%torque del motor electrico
    end
    
    %% 
     
    
    %Transmision
    WGUD(i)=G*WwUD(i); %Vel angular despues del Gear, ws intech velocidad angular de la flecha del motor el�ctrico ecuaci�n 8
    PGUD(i)=TauGUD(i)*WGUD(i); %Potencia mecÁnica A LA SALIDA del eje del motor elÉctrico
    DWGUD(i)=(WGUD(i)-WGUD(i-1))/dt;
    TauCUD(i)=0;
    %% 
    
    %motor electrico
    TEUD(i)=Js*DWGUD(i)+Bv*WGUD(i)+TauGUD(i)+TauCUD(i);%eq 13 intech torque electrmec�nico
    WEUD(i)=P*WGUD(i)/2;%eq 16 intech frecuencia angular del estator REVISAR ESTO
    PEUD(i)=TEUD(i)*WGUD(i); %eq 14 intech potencia eléctrica de la flecha del motor electrico
    %Inversor
    PInvUD(i)=PEUD(i)/etaInv; %potencia a la salida del inversor
    PInvmax=max(PInvUD);
    %% 
    
    %Baterias
    serie2=serie;
    serie = (4*RchaBat*PInvmax)/(npar*VinBat^2);
    serie = max(serie,serie2);
    %seritext = sprintf('%+5.2f',serie2);
    Pload(i)=PInvUD(i)/etaConv; %potencia que debe entrar al convertidor
        %PbatUD(i)=PInvUD(i); %Por la conexi�n en paralelo
    PbatucUD(i)=Pload(i);   %potencia de las baterias y UC %Pbat
     RdiseqBat = (RdisBat*nser)/npar;
    RchaeqBat = (RchaBat*nser)/npar;
    
    %% 
    
    
   
    if PbatucUD(i)>=0
       
        PbatUD(i)=Pload(i); %la potencia de la bateria debe ser igual a la potencia que demanda el convertidor
        IbatdisUD=roots([RdiseqBat -nser*VinBat PbatUD(i)]); %Potencia de la bateria es igual a: potencia de la bateria menos la perdida por la resistencia interna de la bateria
        imIbatdis=imag(IbatdisUD);
                if abs(imIbatdis)>1e-4
                    disp('Corriente no factible, revisar dise�o')
                end
        %x1=2*RdisBat*1e10-VinBat;        %teorema budan-fourier
        %x2=RdisBat*1e10^2-VinBat*1e10+PbatUD(i);        
        iBatUD(i)=min(IbatdisUD); %revisar corriente, esta es corriente que necesita del banco para tracción
        iBatcel(i)= iBatUD(i)/npar;
        vBatUD(i)=nser*VinBat-iBatUD(i)*RdiseqBat;%Vtotal despues de los circuitos eq. serie y paralelo
        vBatUD1(i)=vBatUD(i)/nser;
      
        
    else 
        PbatUD(i)=Pload(i);
        IbatchaUD=roots([RchaeqBat -nser*VinBat PbatUD(i)]);
        imIbatcha=imag(IbatchaUD);
                if abs(imIbatcha)>1e-4
                    disp('Corriente no factible, revisar dise�o')
                end
        iBatUD(i)=min(IbatchaUD); %revisar corriente, esta es corriente que entra al banco del frenado
        iBatcel(i)= iBatUD(i)/npar; %corriente de bateria individual
        vBatUD(i)=nser*VinBat-iBatUD(i)*RchaeqBat;%Vtotal despues de los circuitos eq. serie y paralelo
        vBatUD1(i)=vBatUD(i)/nser;
      
        
    end
    
    SOC(i)=SOC(i-1) - iBatUD(i)*dt/QbatBanco;
    %%SOC(i)=SOC(i-1) - iBatcel(i)*dt/Qbat;
    xUD(i)=vUD(i)+xUD(i-1);
    
%   
    
%% 
    
    if SOC(i)>1 
        SOC(i) = 1;
        %disp('Bateria saturada >1')
    elseif SOC(i)<=0
        SOC(i) = 0;
        iBatUD(i) = 0;
        PbatUD(i)=0;
        %disp('Bateria saturada <0')
        
    end 
    
    if SOC(i)<0.25
       %sprintf('%+12.2f',xUD/1000);
       doblei=i;
       break
    end
    doblei=i;
end
%% 

%UD= doblei;
        nUD= doblei;
        tUD= 0:doblei-1;
        iUCi=0:doblei-1;
pesobateria = nser*npar*pesobat
pesoUC=UCser*UCpar*Pesouc
nser
npar
numbat=nser*npar
serie2
QbatWh=QbatBanco/3600

%xUD=sum(vUD)*dt;
sprintf('Autonomia = %+5.2f km',xUD(doblei)/1000)
%xUDv=trapz(vUD(1),vUD(doblei))


    figure (12)
    plot(tUD,iBatUD,'dc',tUD,iBatcel,'g')
    title('Corriente');
    legend('Total','Celda');
    
    
    figure (13)
    plot(tUD,vBatUD,'dc')
    title('Voltaje');
 
figure (14)
plot(xUD/1000,SOC*100,'o')
ylabel('State of Charge [%]')
xlabel('Range [km]')
title('Estado de carga de las baterias');


 




 
EpCUD=cumtrapz(PpUD)*2.77e-7; %Energia gastada en Wh
EnCUD=cumtrapz(PnUD)*2.77e-7; %Energia candidata a regenerar en Wh
%% 
 
figure(1);
plot(tUDk,vUDk);
title('Velocidad vs tiempo UDDS');
figure(2);
plot(tUD,PGUD,'r', tUD,PEUD,'y');
title('Potencia');
legend( 'potencia mecanica de motor','potencia electrica del motor');
grid on;
figure (4)
plot(tUD,PtUD,'k',tUD,PbatucUD,'c',tUD,PGUD,'g',tUD,PEUD,'r');
title('Potencia');
legend('potencia de traccion','potencia de baterias y UC','potencia en el eje del motor electrico','potencia electrica del motor');
grid on;
figure (6)
plot(tUD,PtUD,'k');
title('Potencia necesaria de tracción UDDS');
grid on;
figure(5);
plot(tUD,PbatucUD,'c*');
title('Potencia');
legend( 'potencia de baterias y UC');
grid on;
figure(3);
plot(tUD,TauGUD,'b',tUD,tauMUD,'c');
title('Torque mecánico necesario UDDS');
grid on;


figure(7);
plot(tUD,iBatUD);
title('Corriente de bateria');
grid on;

figure(8);
plot(tUD/60,WGUD*(60/(2*pi)),'c',tUD/60,WwUD*(60/(2*pi)));
title('VELOCIDAD ÁNGULAR');
legend( 'velocidad angular del motor','velocidad angular de llanta');
grid on;


Torquemaxmotor=max(TauGUD)
Potenciaejemotor=max(PGUD)
