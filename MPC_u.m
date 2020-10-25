m_c=0.493;
m_p=0.312;
I_p=0.00024;
l=0.04;
f1=0.010;
kt=0.11;
R1=10;
r1=0.0335;
g=9.81;
den=I_p*(m_c+m_p)+m_c*m_p*l^2;
A1=[0 1 0 0;0 -(I_p+m_p*l^2)*f1/den m_p^2*g*l^2/den 0;0 0 0 1;0 -m_p*l*f1/den m_p*g*l*(m_c+m_p)/den 0]
B1=[0;(I_p+m_p*l^2)/den;0;m_p*l/den]*(2*kt/(R1*r1))
%A=eye(4)
%B=[1;1;1;1]
C1=[1 0 0 0]
%C1=eye(4)
D=0
%%
R = 1e-4;
RD = 0.1;  %Weight the slew rate - respect actuation bandwidths
Q = 10^8; %Single output
%%
Q1=[1 0 0 0;0 1 0 0;0 0 1000 0;0 0 0 1000]
R1=1;
N1=0;
[K,S1,e] = lqr(A1,B1,Q1,R1,N1)

%%
a=tf(ss(A1-B1*K,B1,C1,D))
b=c2d(a,0.2,'zoh')
c=minreal(ss(b))
%%
A=c.A
B=c.B
C=c.C
%%
N = 100;  %This is the horizon for MPC
Qbar = [];
Rbar = [];
RbarD = [];
Sx = [];
CAB = [];

SU=[];
Su=[];
for ii = 1:N
    Qbar = blkdiag(Qbar,Q);
    Rbar = blkdiag(Rbar,R);
    RbarD = blkdiag(RbarD,RD);
    Sx = [Sx;C*A^ii];
    CAB = [CAB C*A^(ii-1)*B];
end
%CAB
for ii = 1:N
    %SUU=[];
    for jj = 1:ii
        %SUU=[SUU sum(CAB(:,1:ii-jj+1),2)];
        Su(ii,jj)=sum(CAB(1:ii-jj+1));
    end
    %SU = [SUU zeros(4,N-ii)];
    %Su=[Su;SU];
end
Su
Su1=  Su(:,1);
%%
LL = tril(ones(N));
H = 2*(LL'*Rbar*LL+RbarD+Su'*Qbar*Su);
Fu = 2*(diag(LL'*Rbar')'+Su1'*Qbar*Su)';  %Note the trick on Rbar - u(-1) is really a scalar
Fr = -2*(Qbar*Su)';
Fx = 2*(Sx'*Qbar*Su)';
%%
G = [tril(ones(N));-tril(ones(N))];
W0 =5*ones(2*N,1);
S = zeros(2*N,4);
%%
X = [0;0;0;0];
T = 200;
r = 0.1*square([1:T+N+1]/6)
%r=20*ones(T+N+1)
Z = zeros(4,1);
U = 0;
options = optimoptions('quadprog');
options.Display = 'none';
Xact=[];
%x0=zeros(100,1)
for ii = 1:T-1
    Xact(ii,:) = X %For graphing
    f = Fx*X+Fu*U+Fr*r(ii:ii+N-1)';  %Sometimes people hold r(ii) here
    W = W0+[ones(N,1)*-U;ones(N,1)*U];
    Z = quadprog(H,f,G,W+S*X,[],[],[],[],[],options);  %Here is the magic!
    %fun=@(Z)0.5*Z'*H*Z+f'*Z;
    %Z = fmincon(fun,x0,G,W)
    Uopt(ii) = U + Z(1);  %Just use the first item   
    U = Uopt(ii);
    X = A*X+B*U;
end
Xact(ii+1,:) = X;
%%
y=C*Xact'
%%
plot([1:T],y,[1:T],r(1:T))
xlabel('Time steps')
ylabel('Position')
legend('y','r')
r
%%
plot(Uopt)
