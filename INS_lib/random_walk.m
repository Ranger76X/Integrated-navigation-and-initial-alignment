function X = random_walk(ts,t,len,N)

% ts = 0.001;
% t = 0.3;
% len = fix(t/ts);

for i = 1:(1/ts)
    noise = N * randn(1,len)';
    X(1,i) = 0;
    for j = 1:len 
        X(j + 1,i) = exp(-ts/t) * X(j,i) + noise(j,:);
%         noiset(j + 1,i) = noise(j,:);
    end
end
% figure(10);
% plot(X);
% title("Ëæ»úÓÎ×ß");grid on;
end