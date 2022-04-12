desired_d = 2;
sensor_scope_max = 2;
index=1;
x = zeros(102);
y = zeros(102);
for real_d = 0:0.1:4
    x(index) = real_d;
    index = index+1;
    if(real_d<=desired_d)
        cost = (real_d - desired_d)^2/real_d^2;
    else
        cost = (real_d - desired_d)^2/(2*sensor_scope_max-real_d)^2;
    end
    y(index) = cost;
end
plot(x,y);
xlabel("distance/m");
ylabel("cost");

legend("cost function");
