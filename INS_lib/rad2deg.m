function theta = rad2deg(theta_x)

n = size(theta_x);

for i = 1:n(1)
    for j = 1:n(2)
        theta = theta_x * 180 / pi;
    end
end

end