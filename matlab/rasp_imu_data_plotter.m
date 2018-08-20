clear all
close all
x_vec = 0;
y_vec = 0;
z_vec = 0;
time_vec = 0;
t = tcpip('192.168.129.126',5002)
%t = tcpip('172.20.10.12',5002)
fopen(t)
figure
px = plot(x_vec)
ylim([-2 2])
title('X acclerometer')
figure
py = plot(x_vec)
ylim([-2 2])
title('Y acclerometer')
figure
pz = plot(x_vec)
ylim([-2 2])
title('Z acclerometer')
while (1) 
    tic;
    while ~t.BytesAvailable
        if toc>5
            break
        end
        pause(0.1)
    end
    if t.BytesAvailable
        data = char(fread(t,t.BytesAvailable)');
    end
    [time,x,y,z,err] = parseData(data);
    if err 
        break
    end
    x_vec = [x_vec,x];
    y_vec = [y_vec,y];
    z_vec = [z_vec,z];
    time_vec = [time_vec,time];
    set(px,'XData',time_vec,'YData',x_vec)
    set(py,'YData',y_vec)
    set(pz,'YData',z_vec)
    pause(0.1)
    clc
    disp("Samples:")
    disp(length(x_vec))
end
%hold off
disp("Port closed")
fclose(t)


function [time, x,y,z,err] = parseData(data)
    err = false;
    if contains(data,'q')
        err = true;
        time = [];
        x = [];
        y = [];
        z = [];
        return
    end
    data = sscanf(data,'%f',[4 inf])';
    time = data(:,1)';
    x = data(:,2)';
    y = data(:,3)';
    z = data(:,4)';
end