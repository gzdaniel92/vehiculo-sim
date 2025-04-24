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
load('Continuous');
load('Max');
regenmax=-80;
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
