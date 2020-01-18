A = [0 1;-5 -2];
B = [0;3];
C = [0 1];
D = 0;

Ts = 0.25;
sys = ss(A,B,C,D,Ts);
sys.A