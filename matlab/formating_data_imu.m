%% Getting the data
close all
clc
t_data = 0;
for n=1:length(time_vec)
    t_data = [t_data, t_data(n)+time_vec(n)];
end
t_data = t_data(2:end);
t_data = t_data(20:end);
t_data = t_data - t_data(1);
t_diff = time_vec(20:end);
x_data = x_vec(20:end);
y_data = y_vec(20:end);
z_data = z_vec(20:end);
acc = [x_data;y_data;z_data];
plot(t_data,acc)
grid minor
save data.mat t_data t_diff x_data y_data z_data acc
%%
clc
close all
clear all
load data.mat
gravity = [0;0;0];
time_const = 0.001;
for time_const = 0.0001:0.0005:0.002
    gravity = (1-t_diff(1)).*acc(:,1)
    for n = 1:length(t_data)-1
        a = time_const/(time_const+t_diff(n+1));
        gravity = [gravity,a.*gravity(:,n)+(1-a).*acc(:,n+1)];
    end
    hold on
    plot(t_data,gravity(1,:)-acc(1,:))
end
hold off
%%
clc
close all
time_const = 0.001;

for time_const = 1
    a = time_const/(time_const+t_diff(n+1));
    vel = (1-a).*(gravity(:,1)-acc(:,1))*t_diff(1)
    for n = 1:length(t_data)-1
        a = time_const/(time_const+t_diff(n+1));
        vel = [vel,a.*vel(n)+(1-a).*(gravity(:,n+1)-acc(:,n+1))*t_diff(n+1)];
        
    end
    hold on
    vel = vecnorm(vel)
    plot(t_data,vel)
end
hold off