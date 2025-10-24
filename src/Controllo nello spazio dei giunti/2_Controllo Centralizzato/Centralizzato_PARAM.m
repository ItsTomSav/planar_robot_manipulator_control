clc
clear all

%% Variabili
syms   m1   m2        %Masse link
syms   L1   L2        %Lunghezze link
syms   I1   I2        %Inerzie link

syms   d1   d2        %Distanze Joint-CM link

syms   g              %Gravità

syms   th1    th2     %Posizione
syms   th1d   th2d    %Velocità
syms   th1dd  th2dd   %Accelerazione

syms   mm1  mm2       %Masse rotori
syms   Im1  Im2       %Inerzie rotori
syms   Ra1  Ra2       %Resistenze armatura
syms   kt1  kt2       %Costanti di coppia
syms   kv1  kv2       %Costanti di tensione

syms   kr1  kr2       %Rapporti di trasmissione

syms   b1   b2        %Coefficienti di attrito viscoso

syms   u1   u2        %Control input


%% Equazioni di moto

Eq1 = (I1 + m1*d1^2 + Im1*kr1^2 + I2 + Im2 + mm2*L1^2 + m2*(L1^2 + d2^2 + 2*L1*d2*cos(th2)))*th1dd +...
      + (I2 + kr2*Im2 + m2*(d2^2 + L1*d2*cos(th2)))*th2dd - 2*m2*L1*d2*sin(th2)*th1d*th2d - m2*L1*d2*sin(th2)*th2d*th2d + ...
      + g*(m1*d1 + mm2*L1 + m2*L1)*cos(th1) + m2*d2*g*cos(th1+th2) + (b1 + kr1*kt1*kv1*kr1*(1/Ra1))*th1d == u1;

Eq2 = (I2 + kr2*Im2 + m2*(d2^2 + L1*d2*cos(th2)))*th1dd + (I2 + m2*d2^2 + Im2*kr2^2)*th2dd + ...
      + m2*L1*d2*sin(th2)*th1d*th1d + m2*d2*g*cos(th1+th2) + (b2 + kr2*kt2*kv2*kr2*(1/Ra2))*th2d == u2;


%% Risoluzione equazioni
S = solve([Eq1, Eq2], [th1dd, th2dd]);

TH1DD = S.th1dd;
TH2DD = S.th2dd;


%% Risolviamo le equazioni con Simulink
% TF_Sim = true;
% if(true == TF_Sim)
%     MODEL1 = 'EOM_SIM1';
%     %Usiamo exist=4 per verificare che esista un modello
%     %Simulink nel folder.
%     if(4 == exist(MODEL1))
%         close_system(MODEL1, 0); %Chiudo il sistema senza salvare
%         delete(MODEL1);
%     end
%     new_system(MODEL1)
%     open_system(MODEL1)   
% 
%     %Mettiamo entrambe le equazioni nella funzione Matlab in Simulink
%     matlabFunctionBlock( [MODEL1,'/Thetadd_eq'], TH1DD, TH2DD, ...
%                          'Optimize', false, ...        %Per non far semplificare le espressioni da Simulink
%                          'Outputs', {'theta1_DD', 'theta2_DD'});     
% end


%% Parametri Manipolatore

L_1 = 1;      L_2 = 1;          %Lunghezze link (m)
d_1 = 0.5;    d_2 = 0.5;        %Distanze dei baricentri dai giunti (m)

m_1 = 50;     m_2 = 50;         %Masse link (Kg)
I_1 = 10;     I_2 = 10;         %Inerzie link (Kg*m^2)

mm_1 = 5;     mm_2 = 5;         %Masse rotori (Kg)
Im_1 = 0.01;  Im_2 = 0.01;      %Inerzie rotori (Kg*m^2)
kr_1 = 100;   kr_2 = 100;       %Rapporti di trasmissione

gr = 9.81;                     %Gravità (m/s^2)

b_1 = 0.01; b_2 = 0.01;     %Coefficienti di attrito viscoso del motore

Ra_1 = 10;    Ra_2 = 10;      %Resistenze di armatura

kt_1 = 2;     kt_2 = 2;       %Costanti di coppia
kv_1 = 2;     kv_2 = 2;       %Costanti di tensione


%% Condizioni iniziali
th1_0  =  deg2rad(30);    % rad
% th1_0  =  0;    % rad
th1d_0 =  0;              % rad/sec 

th2_0  =  deg2rad(45);    % rad
% th2_0  =  0;    % rad
th2d_0 =  0;              % rad/sec


%% Elementi costanti

%g(q)
g1_1 = gr*(m_1*d_1 + mm_2*L_1 + m_2*L_1);  % *cos(th1)
g1_2 = m_2*d_2*gr;  % *cos(th(1+2))
g2 = m_2*d_2*gr;    % *cos(th(1+2))

%B(q)
b11 = I_1 + m_1*d_1^2 + Im_1*kr_1^2 + I_2 + Im_2 + mm_2*L_1^2 + m_2*(L_1^2 + d_2^2);
b22 = I_2 + m_2*d_2^2 + Im_2*kr_2^2;
b12 = I_2 + kr_2*Im_2 + m_2*d_2^2;

delta_b11 = 2*m_2*L_1*d_2;  % *cos(th2)
delta_b12 = m_2*L_1*d_2;    % *cos(th2)

%c(q,qd)
h = -m_2*L_1*d_2;  % *sin(th2)*th...

%F
F1 = b_1 + kr_1*kt_1*kv_1*kr_1*(1/Ra_1);
F2 = b_2 + kr_2*kt_2*kv_2*kr_2*(1/Ra_2);



%% RIFERIMENTO

% Per il controllo PD con compensazione di gravità.
% Postura costante
qd = [deg2rad(60); deg2rad(20)];


% Per il controllo a dinamica inversa e controllo robusto.
% Introduciamo un profilo trapezoidale di velocità

% Parametri per traiettorie:

tc = 0.6;                  
qc_d = 1;                    %Velocità di crociera (max)
q_in = 0;   q_fin = 1;       %Posizione iniziale e finale

tf = tc + (q_fin-q_in)/qc_d; %Tempo finale

qc_dd = qc_d/tc;             %Accelerazione massima


%% Costanti di controllo

disp('Imposta il tipo di controllo(selezionare un numero tra 1 e 3):')
disp('(1) Controllo PD con compensazione di gravità')
disp('(2) Controllo a dinamica inversa')
disp('(3) Controllo robusto')

scelta = input('Inserisci il numero corrispondente al tipo di controllo desiderato: ');

switch scelta
    case 1
        disp('Hai scelto il controllo PD con compensazione di gravità.')

        KP = 3750*eye(2);
        KD = 750*eye(2);
    
        run("1_Controllo PD con compensazione di gravita\Controllo_PD_Comp.slx")

    case 2
        disp('Hai scelto il controllo a dinamica inversa.')

        KP = 25*eye(2);
        KD = 5*eye(2);

        run("2_Controllo a dinamica inversa\Controllo_din_inversa.slx")

    case 3
        disp('Hai scelto il controllo robusto.')

        KP = 25*eye(2);
        KD = 5*eye(2);

        %Calcolo della robustezza


        %Matrice P
        P = eye(4);

        %Matrice D
        D = zeros(4,2);
        D(3,1) = 1;
        D(4,2) = 1;

        %Matrice H
        H = zeros(4,4);
        H(1,3) = 1;
        H(2,4) = 1;
        
        %Matrice K
        K = [KP KD];
        
        H_tilde = H - (D*K);
        
       
        %Definisco gli elementi della matrice Q come variabili qi
        syms q1 q2 q3 q4
        syms q5 q6 q7 q8
        syms q9 q10 q11 q12
        syms q13 q14 q15 q16
        Qx = [q1  q2  q3  q4;
              q5  q6  q7  q8;
              q9  q10 q11 q12;
              q13 q14 q15 q16];

        %Equazione da risolvere
        equaz = (H_tilde')* Qx + Qx * H_tilde == -P;
        
        %Creo un vettore che contiene gli elementi della matrice Q
        vectq = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16];
        
        %Risolvo l'equazione rispetto alle variabili qi
        tom = solve (equaz,vectq);

        %Definisco la matrice Q sostituendo i qi con i valori trovati
        vecttom = [tom.q1 tom.q2 tom.q3 tom.q4 tom.q5 tom.q6 tom.q7 tom.q8 tom.q9 tom.q10 tom.q11 tom.q12 tom.q13 tom.q14 tom.q15 tom.q16];
        Q_sym = subs(Qx,vectq,vecttom);
        Q = double(Q_sym);
        
        z1 = (D')*(Q);
        
        ro = 70;
        epsilon = 0.004;


        run("3_Controllo Robusto\Controllo_Robusto.slx")
        
    otherwise
        disp('Selezione non valida.')
end