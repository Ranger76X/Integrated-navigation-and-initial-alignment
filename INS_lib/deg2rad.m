function theta = deg2rad(theta_x)
n = size(theta_x);

for i = 1:n(1)
    for j = 1:n(2)
        theta(i,j) = theta_x(i,j) / 180 * pi;
    end
end

end