clc
clear all

%% PARAMETRI MANIPOLATORE 
% I due bracci sono stati scelti uguali per evidenziare i contributi di 
% interazione dinamica tra i due giunti.

L1 = 1;      L2 = 1;          %Lunghezze link (m)
d1 = 0.5;    d2 = 0.5;        %Distanze dei baricentri dai giunti (m)

m1 = 50;     m2 = 50;         %Masse link (Kg)
I1 = 10;     I2 = 10;         %Inerzie link (Kg*m^2)

mm1 = 5;     mm2 = 5;         %Masse rotori (Kg)
Im1 = 0.01;  Im2 = 0.01;      %Inerzie rotori (Kg*m^2)
kr1 = 100;   kr2 = 100;       %Rapporti di trasmissione

g = 9.81;                     %Gravità (m/s^2)


%% PARAMETRI ATTUATORI
Fm1 = 0.01; Fm2 = 0.01;     %Coefficieni di attrito viscoso del motore
% Si suppone che Fm sia molto più piccolo del coefficiente di attrito
% elettrico, quindi trascurabile

Ra1 = 10;    Ra2 = 10;      %Resistenze di armatura

kt1 = 2;     kt2 = 2;       %Costanti di coppia
kv1 = 2;     kv2 = 2;       %Costanti di tensione

Tm = (Ra1*Im1)/(kt1*kv1);   %Costante di tempo fdt di motore
km = 1/kv1;                 %Guadagno fdt motore


%% ELEMENTI COSTANTI

%Consideriamo la matrice delle inerzie B e la dividiamo in B_ e DeltaB
b11 = I1 + m1*d1^2 + Im1*kr1^2 + I2 + Im2 + mm2*L1^2 + m2*(L1^2 + d2^2);
b22 = I2 + m2*d2^2 + Im2*kr2^2;
b12 = I2 + kr2*Im2 + m2*d2^2;

delta_b11 = 2*m2*L1*d2;  % *cos(th2)
delta_b12 = m2*L1*d2;    % *cos(th2)

%Consideriamo il vettore di gravità g
g1_1 = g*(m1*d1 + mm2*L1 + m2*L1);  % *cos(th1)
g1_2 = m2*d2*g;  % *cos(th(1+2))
g2 = m2*d2*g;    % *cos(th(1+2))

%Consideriamo la matrice C
h = -m2*L1*d2;  % *sin(th2)*th...


%% RIFERIMENTO
% Introduciamo un profilo trapezoidale di velocità

% Parametri traiettoria:

tc = 0.6;                  
qc_d = 1;                    %Velocità di crociera (max)
q_in = 0;   q_fin = 1;       %Posizone iniziale e finale


tf = tc + (q_fin-q_in)/qc_d; %Tempo finale

qc_dd = qc_d/tc;             %Accelerazione massima


%% PARAMETRI CONTROLLORI
%{ 
Introduciamo tre diversi controlli:
1) Retroazione di posizione;
2) Retroazione di posizione e velocità;
3) Retroazione di posizione, velocità e accelerazione;

I valori dei parametri dei controllori sono stati ottimizzati attraverso 
la tecnica del "trial and error".
%}

disp('Imposta il tipo di controllo (selezionare un numero tra 1 e 3):')
disp('(1) Feedback Control')
disp('(2) Feedforward Compensation')
disp('(3) Computed Torque Feedforward Control')

tipo_controllo = input('Inserisci il numero corrispondente al tipo di controllo desiderato: ');

if tipo_controllo ~= 1 && tipo_controllo ~= 2 && tipo_controllo ~= 3
    disp('Selezione non valida.')
    return
end

fprintf('\n');
disp('Imposta il tipo di controllo(selezionare un numero tra 1 e 3):')
disp('(1) Controllo di posizione')
disp('(2) Controllo di posizione e velocità')
disp('(3) Controllo di posizione, velocità e accelerazione')

scelta = input('Inserisci il numero corrispondente al tipo di controllo desiderato:\n ');

switch scelta
    case 1

        % (1) Posizione
        KP = 5;                            %Costante proporzionale di posizione
        TP = 50*Tm;                        %Costante di tempo fdt controllore di posizione
                                           %(Per la stabilità, si deve avere che TP>>Tm)
        kTP = 1;                           %Costante di trasduzione di posizione

        if tipo_controllo == 1
             disp('Hai scelto retroazione di posizione.')
            run("1_Feedback Control\Controllo_Indipendente_POS.slx")
        elseif tipo_controllo == 2
            disp(' Hai scelto retroazione di posizione con azione di compensazione in avanti')
            run("2_Feedforward Compensation\Contr_Indipendente_Avanti_POS_.slx")
        elseif tipo_controllo == 3
             disp('Hai scelto retroazione di posizione con coppia precalcolata.')
            run("3_Computed Torque Feedforward Control\Contr_Indipendente_Avanti_CompTorque_POS_.slx")
        end

    case 2

       % (2) Posizione + Velocità
        KP = 5;     KV = 10;               %Costante proporzionale di posizione e velocità
        TV = Tm;                           %Costante di tempo fdt controllore di velocità
                                           %(Per cancellare gli effetti del polo reale del motore
                                           % in s = −1/Tm poniamo TV=Tm)
        kTP = 1;    kTV = 1;               %Costanti di trasduzione di posizione e velocità 

        if tipo_controllo == 1
            disp('Hai scelto retroazione di posizione e di velocità.')
            run("1_Feedback Control\Controllo_Indipendente_POS_VEL.slx")
        elseif tipo_controllo == 2
             disp(' Hai scelto retroazione di posizione e velocità con azione di compensazione in avanti.')
            run("2_Feedforward Compensation\Contr_Indipendente_Avanti_POS_VEL.slx")
        elseif tipo_controllo == 3
             disp('Hai scelto retroazione di posizione e velocità con coppia precalcolata.')
            run("3_Computed Torque Feedforward Control\Contr_Indipendente_Avanti_CompTorque_POS_VEL.slx")
        end

    case 3

        % (3) Posizione + Velocità + Accelerazione
        KP = 5;     KV = 10;     KA = 2;   %Costante proporzionale di posizione e velocità
        TA = Tm;                           %Costante di tempo fdt controllore di accelerazione
                                           %(Per cancellare gli effetti del polo reale del motore
                                           % in s = −1/Tm poniamo TA=Tm)
        kTP = 1;    kTV = 1;    kTA = 1;   %Costanti di trasduzione di posizione e velocità

        if tipo_controllo == 1
            disp('Hai scelto retroazione di posizione, di velocità e di accelerazione.')
            run("1_Feedback Control\Controllo_Indipendente_POS_VEL_ACC.slx")
        elseif tipo_controllo == 2
            disp(' Hai scelto retroazione di posizione, velocità e accelerazione con azione di compensazione in avanti.')
            run("2_Feedforward Compensation\Contr_Indipendente_Avanti_POS_VEL_ACC.slx")
        elseif tipo_controllo == 3
            run("3_Computed Torque Feedforward Control\Contr_Indipendente_Avanti_CompTorque_POS_VEL_ACC.slx")
            disp('Hai scelto retroazione di posizione, di velocità e di accelerazione con coppia precalcolata.')
        end
        
    otherwise
        disp('Selezione non valida.')
end