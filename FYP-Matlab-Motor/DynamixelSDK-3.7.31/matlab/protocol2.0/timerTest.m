clc
clear all

tStart = tic;           % pair 2: tic
n = 10;
T = zeros(1,n);
for i = 1:n
    A = rand(12000,4400);
    B = rand(12000,4400);
    tic         % pair 1: tic
    C = A.*B;
    T(i)= toc;  % pair 1: toc
end
tMul = sum(T)