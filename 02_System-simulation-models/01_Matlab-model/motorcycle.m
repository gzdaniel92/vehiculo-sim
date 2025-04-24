close all
clear all
clc
restoredefaultpath
addpath(genpath('D:\vehiculo-sim'));
%% Carga los datos de los diferentes ciclos de manejo.
drive_cycle=load('FTP.txt','-ASCII');

YUDDS=load('UDDS.txt','-ASCII');    
YUDDS1=load('uddsextendido.txt','-ASCII');
YNYCC=load('NYCCCOL.TXT','-ASCII');
YFTP=load('FTP.txt','-ASCII');
YZol=load('ZOLDER.txt','-ASCII');
%% step time simulink
sim_dt=0.1;
%% datos del motor
speed=load('spd_map.txt','-ASCII');
loss_pow=load('losses_w.txt','-ASCII');
torque=load('trq_vec.txt','ASCII');
effi=load('gvm_eff_map1.txt','ASCII');
EFF=load('gvm_eff_map.txt','ASCII');
cont=load('continuous.txt','ASCII');
peak=load('peak.txt','ASCII');

%subrutina de calculo de ciclos
YUDDS=drive_cycle;%para asignar el primer ciclo
%% 

nUD=size(YUDDS);%para el valor final para la indexacion del siguiente ciclo
nCiclos=1;%numero de ciclos que se requieren<------------------------------------------
for i=1:nCiclos-1
    YUDDS(end+1:(end+nUD),:)=drive_cycle;
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
%% Par�metros del vehiculo
m=260+00 ; %kg
g=9.81 ; %m/s
alpha=0*pi/180; %radianes
Alpha=sin(alpha);
ro=1.2041; %kg/m3 Densidad del aire
Af=0.72; %m2 area frontal
Cd=0.84; %Coeficiente de arrastre (adimensional)
dt=1; %s Tiempo de muestreo del patr�n de manejo
rw=(15/2)*0.0254+.65*0.145; %radio de llanta 145/65 R15
Crr=0.005;
RollresA=0.005;
RollresB=(3.6/100)*RollresA;
%% Vehicle coefficients
At=m*9.81*RollresA;
Bt=m*9.81*RollresB;
Ct=0.5*Cd*Af*ro;
%% Par�metros del motor eléctrico
load('motor_gvm210-150.mat');
Js=2.825e-02; %Momento de inercia 
Bv=0.1459; %Coeficiente de fricci�n viscosa
P=12; %Número de polos
%% Par�metros del sistema de tracci�n 
FD=2;
etaFD=0.98;
G=[10 3]; %Depende del sistema de tracci�n. gear ratio PARA UDDS Y US06 ES 2.5
etaG=0.95; %Eficiencia del engrane
%% Par�metros del inversor
etaInv=0.95; %Eficiencia del inversor
%Par�metros del convertidor DC/DC
etaConv = 0.95;
%% Parametros del ultracapacitor
UCser=27; %UC en serie
UCpar=4; %UC en paralelo
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
%% UC para modelo simulink 
Vuc=48;
Iucmax=140;


C=165;
Rsc=0.15e-3;
RL=0.000000001;

SoCucmax=0.95;
SoCucmin=0.1;

V_UC=3;
C_UC=3400;

nsera=27;
npara=4;

nserf=108;
nparf=1;


Rsc_A=Rsc*nsera+(1/(npara/Rsc));
Rsc_sa_bat=Rsc*nserf;
Rsc_sa_uc=Rsc*nsera+(1/(npara/Rsc));
Rsc_F=Rsc*nserf;

Vuc_fa=nsera*3;
Vuc_sa_bat=nserf*3;
Vuc_sa_uc=nsera*3;
Vuc_free=nserf*3;

C_fa=1/(nsera/(npara*C_UC));
C_sa_bat=1/(nserf/(nparf*C_UC));
C_sa_uc=1/(nsera/(npara*C_UC));
C_free=1/(nserf/(nparf*C_UC));
%% Parámetros de la bateria (cambiar para cada simulación)
RdisBat=0.55e-3; %ohms
RchaBat=0.55e-3;   %ohms
VinBat=3.2; %voltage nominal
nser= 19; %%bater�as en serie 
npar= 4; %%baterias en paralelo
Cmax=2;

nserfa=19;
nparfa=4;

nsersa_bat=19;
nparsa_bat=4;

nsersa_UC=76;
nparsa_UC=1;

nser_free=76;
npar_free=1;

SoCbatmin=0.20;
SoCbatmax=0.80;
% Imax=Qbat*Cmax;
DoD=1-0.8;
SOC (1)= 1 - DoD;
Qbat =26 ;%Ah
Enertotbat=Qbat*npar
QbatBanco = Qbat*npar*3600; %Qbat*npar/nser*3600
Pcellbat=Qbat*VinBat;
Ptotbat=Pcellbat*nser*npar
Ptotbatneg=-1*Ptotbat;
vBatUD(1)=VinBat;
vBatUD1(1)=VinBat*nser;
pesobat=0.387;
% LF50 las de potencia industrial azules pequeñas
SOC_BP=[0      0.05   0.1    0.15   0.20   0.25   0.30   0.35   0.40   0.45   0.50   0.55   0.60   0.65   0.70   0.75   0.80   0.85   0.90   0.95   1.00];
Voc_BP=[2.9628 3.187  3.2093 3.2288 3.2496 3.2632 3.2796 3.2927 3.2942 3.2952 3.2961 3.2976 3.3004 3.3144 3.3327 3.3336 3.3342 3.3355 3.3361 3.3373 3.4071];
Ri_BP= [0.8847 0.8127 0.7900 0.8040 0.7920 0.7727 0.7667 0.7667 0.7460 0.7193 0.7093 0.7027 0.6860 0.6680 0.6433 0.6180 0.6160 0.6060 0.6080 0.6093 0.8187]*10^(-3);
%kokam
Voc_BP=[2.7	3.185	3.67	3.699444444	3.728888889	3.758333333	3.787777778	3.817222222	3.846666667	3.876111111	3.905555556	3.935	3.964444444	3.993888889	4.023333333	4.052777778	4.082222222	4.111666667	4.141111111	4.170555556	4.2];
Ri_BP= (16/7)*[0.8847 0.8127 0.7900 0.8040 0.7920 0.7727 0.7667 0.7667 0.7460 0.7193 0.7093 0.7027 0.6860 0.6680 0.6433 0.6180 0.6160 0.6060 0.6080 0.6093 0.8187]*10^(-3);


iR_bp=[Ri_BP' Ri_BP']';
Temp_bp=[293.15 313.15];
OCV=[Voc_BP' Voc_BP']';
SOC_defbp=[SOC_BP' SOC_BP']';


%% C�lculo de la demanda de potencia
ftUD(1)=0;
ftNY(1)=0;
ftFTP(1)=0;

serie2=0;%solamente es una variable de apoyo
serie=0;
gs(1)=1;
gear(1)=G(1);
m = m+(npar*nser*pesobat)+(Pesouc*UCser*UCpar)  %kg con baterias

%% 
for i=2:nUD
    dist(1)=0;
   dist(i)=vUD(i)+dist(i-1); 
end
distancia=max(dist);
sprintf('largo del recorrido = %+5.2f km',distancia/1000)



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
    Tor(i)=PtUD(i)./WwUD(i); %torque de tracción
    if PtUD(i)>0
       PpUD(i)=PtUD(i);
       PnUD(i)=0;
       Tau(i,:)=(1/etaG)*tauMUD(i)./(G); %torque mecánico del motor electrico
       WG(i,:)=G*WwUD(i); %Vel angular despues del Gear, ws intech velocidad angular de la flecha del motor el�ctrico ecuaci�n 8
       effp(i,:)=interp2(speed*(2*pi/60),torque,effi,WG(i,:),Tau(i,:));
        [com(i),gs(i)]=max(effp(i,:));
    else
        PnUD(i)=PtUD(i);
        PpUD(i)=0;
        Tau(i,:)=(etaG)*tauMUD(i)./(G);%torque mecnánico del motor electrico
        WG(i,:)=G*WwUD(i); %Vel angular despues del Gear, ws intech velocidad angular de la flecha del motor el�ctrico ecuaci�n 8
        effp(i,:)=interp2(speed*(2*pi/60),torque,effi,WG(i,:),Tau(i,:));
        [com(i),gs(i)]=max(effp(i,:));
        %gs(i)=gs(i-1);
    end
    if com(i)==0;
   
    gs(i)=gs(i-1);
    WGUD(i)=WG(i,gs(i));
    TauGUD(i)=Tau(i,gs(i));
    else
    WGUD(i)=WG(i,gs(i));
    TauGUD(i)=Tau(i,gs(i));    
    end
    
    if     gs(i)==1; gear(i)=G(1);
    elseif gs(i)==2; gear(i)=G(2);
    elseif gs(i)==3; gear(i)=G(3);
    elseif gs(i)==4; gear(i)=G(4);
    elseif gs(i)==5; gear(i)=G(5);
    elseif gs(i)==6; gear(i)=G(6);
    elseif gs(i)==7; gear(i)=G(5);
    elseif gs(i)==8; gear(i)=G(6);  
    end
    %% Transmision
   
    PGUD(i)=TauGUD(i).*WGUD(i); %Potencia mecÁnica A LA SALIDA del eje del motor elÉctrico
    
    
    
    %aqui hay que elegir el    speed, torque,EFF,
    
    
    
    DWGUD(i)=(WGUD(i)-WGUD(i-1))/dt;
    TauCUD(i)=0;
    %% 
    
    %motor electrico
    TEUD(i)=Js*DWGUD(i)+Bv*WGUD(i)+TauGUD(i)+TauCUD(i);%eq 13 intech torque electrmec�nico
    WEUD(i)=P*WGUD(i)/2;%eq 16 intech frecuencia angular del estator 
    PEUD(i)=TEUD(i).*WGUD(i); %eq 14 intech potencia eléctrica de la flecha del motor electrico
    %Inversor
    PInvUD(i)=PEUD(i)/etaInv; %potencia a la salida del inversor
    PInvmax=max(PInvUD);
    %% Baterias
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
        Vs(i)=PbatUD(i)/iBatUD(i);
        vBatUD(i)=nser*VinBat-iBatUD(i)*RdiseqBat;%Vtotal despues de los circuitos eq. serie y paralelo
        vBatUD1(i)=vBatUD(i)/nser;
      
        
    else 
        PbatUD(i)=Pload(i);
%         IbatchaUD=roots([RchaeqBat -nser*VinBat PbatUD(i)]);
        IbatchaUD=roots([RchaeqBat -nser*VinBat PbatUD(i)]);
        imIbatcha=imag(IbatchaUD);
                if abs(imIbatcha)>1e-4
                    disp('Corriente no factible, revisar dise�o')
                end
        iBatUD(i)=min(IbatchaUD); %revisar corriente, esta es corriente que entra al banco del frenado
        iBatcel(i)= iBatUD(i)/npar; %corriente de bateria individual
        vBatUD(i)=nser*VinBat-iBatUD(i)*RchaeqBat;%Vtotal despues de los circuitos eq. serie y paralelo
        vBatUD1(i)=vBatUD(i)/nser;
        Vs(i)=PbatUD(i)/iBatUD(i);
      
        
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


    figure (1)
    plot(tUD,iBatUD,'dc-',tUD,iBatcel,'g')
    title('Corriente');
    legend('Total','Celda');
    
    
    figure (2)
    plot(tUD,vBatUD,'dc-',tUD,Vs,'r--')
    title('Voltaje');
 
    figure (3)
    plot(xUD/1000,SOC*100,'o--')
    ylabel('State of Charge [%]')
    xlabel('Range [km]')
    title('Estado de carga de las baterias');

EpCUD=cumtrapz(PpUD)*2.77e-7; %Energia gastada en Wh
EnCUD=cumtrapz(PnUD)*2.77e-7; %Energia candidata a regenerar en Wh
%% 
 
figure(4);
plot(tUDk,vUDk);
title('Velocidad vs tiempo');

figure(5);
plot(tUD,PGUD,'r', tUD,PEUD,'y');
title('Potencia');
legend( 'potencia mecanica de motor','potencia electrica del motor');
grid on;

figure (6)
plot(tUD,PtUD,'k',tUD,PbatucUD,'c',tUD,PGUD,'g',tUD,PEUD,'r');
title('Potencia');
legend('potencia de traccion','potencia de baterias y UC','potencia en el eje del motor electrico','potencia electrica del motor');
grid on;

figure (7)
plot(tUD,PtUD,'k');
title('Potencia necesaria de tracción UDDS');
grid on;

figure(8);
plot(tUD,PbatucUD,'c*--');
title('Potencia');
legend( 'potencia de baterias y UC');
grid on;

figure(9);
plot(tUD,TauGUD,'b.:')%,tUD,tauMUD,'c');
legend('torque del motor')%,'torque de tracción')
title('Torque mecánico necesario');
grid on;

figure(10);
plot(tUD,iBatUD);
title('Corriente de bateria');
grid on;

figure(11);
plot(tUD,WGUD*(60/(2*pi)),'b.:')%,tUD/60,WwUD*(60/(2*pi)));
title('VELOCIDAD ÁNGULAR en RPM');
legend( 'velocidad angular del motor')%,'velocidad angular de llanta');
grid on;
%% stats bar
stats=[(WGUD*(60/(2*pi))).', TauGUD.'];% vectores de velocidad y torque de la flecha del motor eléctrico
Torquemaxmotor=max(TauGUD)
Potenciaejemotor=max(PGUD)




torque1=sort(torque.');
torque2=-270:5:270;




figure(13)
hist3(stats,'LineStyle',':','FaceColor','interp','CdataMode','auto','Ctrs',{0:200:8000 -270:5:270})
xlabel('speed')
ylabel('torque')
colorbar
count1=(hist3(stats,'LineStyle',':','FaceColor','interp','CdataMode','auto','Ctrs',{speed torque1})).';%matriz de como estan acomodados los valores
%%
Torquem=TauGUD.';% vector de torque de la flecha del motor eléctrico
velom=(WGUD*(60/(2*pi))).';% vector de velocidad de la flecha del motor eléctrico

figure(14)
contourf(speed, torque,EFF, [0.6:0.05:0.8, 0.82:0.02:0.9, 0.91:0.01:0.98],'ShowText','on','LabelSpacing',50,'HandleVisibility','off');
colormap jet
hold on; grid on;


hist3(stats,'LineStyle',':','CdataMode','auto','Ctrs',{speed torque1})
xlabel('speed')
ylabel('torque')
colorbar
%view(2)


count=histcounts2(stats(:,1),stats(:,2),speed,torque1); 
count=count.';
%% efficieny statistical points change velom and Torquem
EFF1=EFF; %mapa de eficiencias del motor GVM-150
EFF(isnan(EFF))=0; %cambia todos los datos donde no existe lectura de eficiencia por cero
count1m=changem(count1,NaN,0.0);%cambia los valores donde no hay torque por NaN

Vq=interp2(speed,torque,EFF,velom,Torquem);%interpolación en 2D

Mult=EFF.*count1m;
neff=Mult./(count1m); 

veff=neff(:); %pone en orden
vcount=count1m(:); %pone en orden el vector

vplot=[veff vcount];

vplot1=sortrows(rmmissing(vplot));
%vplot1=changem(vplot1,0.7,0.0);

[Xu1,~,Ic2] = unique(vplot1(:,1), 'stable');
Tally = accumarray(Ic2, vplot1(:,2));
vplotr = [Xu1 Tally];
L=length(vplotr(:,1));

figure(15)
bar(1:L,vplotr(:,2),1.0)
title('statistical plots')
xlabel('Efficiency')
ylabel('Frequency')
set(gca, 'XTick', 1:L, 'XTickLabel', vplotr(:,1));
grid on
%%
figure(16)
plot3(tUD,WGUD*(60/(2*pi)),TauGUD,'b.:')
xlabel('time')
ylabel('velocity[rpm]')
zlabel('torque')
grid on

figure(17)
plot3(Torquem,velom,Vq,'b.:')
xlabel('torque')
ylabel('velocity[rpm]')
zlabel('efficiency')
grid on

aafinalv=[tUD.',velom,Torquem,Vq];

figure(18)
plot(tUD,gs)
hold on
plot(tUDk,vUDk/10);
hold on
grid on

%% Poner esta grafica en la tesis (estadistica) change WGUD and TAUGUD
load('Continuous');
load('Max');
figure (12)

contourf(speed, torque,effi, [0.6:0.05:0.8, 0.82:0.02:0.9, 0.91:0.01:0.98],'ShowText','on','LabelSpacing',50,'HandleVisibility','off');
colormap turbo
hold on; 
grid on;
plot(WGUD*(60/(2*pi)),TauGUD,'dg')
plot(Continuous(:,1),Continuous(:,2),'-k','LineWidth',4)
plot(Continuous(:,1),-Continuous(:,2),'-k','LineWidth',4)
plot(Max(:,1),Max(:,2),'-k','LineWidth',4)
plot(Max(:,1),-Max(:,2),'-k','LineWidth',4)
title('Visitation points')

trq_motor=[tUD; TauGUD]';
gearc=[(1:nUD); gear]';



