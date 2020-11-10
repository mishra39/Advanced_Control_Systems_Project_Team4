%% Connect to the HC-06 module
close all;
bt_connected = 0; % change it to 1 after connecting to bluetooth
if (bt_connected == 0)
    clear; 
    bt = Bluetooth("HC-06",1); % connects to the bluetooth module as a serial communication object
    fopen(bt); % initialize the serial object
end
%%
m_c=0.493;
m_p=0.312;
I_p=0.00024;
l=0.04;
f1=0.01;
kt=0.11;
R1=10;
r1=0.0335;
g=9.81;
den=I_p*(m_c+m_p)+m_c*m_p*l^2;
A1=[0 1 0 0;0 -(I_p+m_p*l^2)*f1/den m_p^2*g*l^2/den 0;0 0 0 1;0 -m_p*l*f1/den m_p*g*l*(m_c+m_p)/den 0];
B1=[0;(I_p+m_p*l^2)/den;0;m_p*l/den]*(2*kt/(R1*r1));
%A=eye(4)
%B=[1;1;1;1]
C1=[1 0 0 0];
%C1=eye(4)
D=0;
R = 1/100;
RD = 0.1;  %Weight the slew rate - respect actuation bandwidths
Q = 10^8;  %Single output

Q1=[1 0 0 0;0 1 0 0;0 0 5000 0;0 0 0 5000];
R1=1;
N1=0;
[K,S1,e] = lqr(A1,B1,Q1,R1,N1);


a=tf(ss(A1-B1*K,B1,C1,D));
b=c2d(a,0.2,'zoh');
c=minreal(ss(b));

A=c.A;
B=c.B;
C=c.C

N = 10;  %This is the horizon for MPC
Qbar = [];
Rbar = [];
RbarD = [];
Sx = [];
Su = [];
CAB = [];
for ii = 1:N
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
    RbarD = blkdiag(RbarD,RD);
    Sx = [Sx;C*A^ii];
    CAB = [CAB;C*A^(ii-1)*B];
end
for ii = 1:N
    for jj = 1:ii
        Su(ii,jj) = sum(CAB(1:ii-jj+1));
    end
end
Su1=  Su(:,1);

LL = tril(ones(N));
H = 2*(LL'*Rbar*LL+RbarD+Su'*Qbar*Su);
Fu = 2*(diag(LL'*Rbar')'+Su1'*Qbar*Su)';  %Note the trick on Rbar - u(-1) is really a scalar
Fr = -2*(Qbar*Su)';
Fx = 2*(Sx'*Qbar*Su)';

G = [tril(ones(N));-tril(ones(N))];
W0 =12*ones(2*N,1);
S = zeros(2*N,4);

X = [0;0;0;0];
T = 200;
%r = 0.1*square([1:T+N+1]/6);
%r=0.1:0.01:0.5;
r=0.1*ones(1,N*100);
Z = zeros(4,1);
U = 0;
options = optimoptions('quadprog');
options.Display = 'none';
%% Stream and send data to HC-06

% endless loop to keep streaming data from the robot
ii=1;
Xact=[];
Ulqr=[];
while (ii<T)
    flushinput(bt); % flush the buffer stored till now
    fscanf(bt); % flushing buffer clips the data stream so have to discard the next input
    rawdata = fscanf(bt); % Store the current data streamed from the robot
    
    X = parsedata(rawdata); % parse the raw data and store in a numeric array
    Ulqr=[Ulqr X(5,1)];
    X=X(1:4,1)
  
    f = Fx*X+Fu*U+Fr*(r(ii:ii+N-1))';  %Just trying to track now
    W = W0+[ones(N,1)*-U;ones(N,1)*U];
    Z = quadprog(H,f,G,W+S*X,[],[],[],[],[],options);  %Here is the magic!
   
    Uopt(ii) = U + Z(1);  %Just use the first item   
    U = Uopt(ii);
    Xact(ii,:) = X;
    flushoutput(bt); % flush output buffer
    fprintf(bt,num2str(U)) % send the output variables to the robot in a string forma
    ii=ii+1;
    %X = A*X+B*U;
    
end
Xact(ii+1,:) = X;
flushoutput(bt);
fprintf(bt,num2str(0));
%y=C*Xact';
figure();
plot(Xact(:,1))
hold on
plot(r(1:T))
hold off
ylim([-0.3,0.3]);
legend('Pos', 'Reference');
xlabel('Step')
ylabel('Position(rad/s)')
title('MPC Regulation Control')

figure()
plot(Uopt);
hold on;
plot(Ulqr);
hold off;
legend('u_MPC', 'u_total');
ylabel('Control')