%rpi = tcpclient('192.168.129.126',5002)
clear all
close all
x_vec = 0;
y_vec = 0;
z_vec = 0;
time_vec = [];
t = tcpip('192.168.129.126',5002)
%t = tcpip('172.20.10.12',5002)
fopen(t)
figure
hold on
p = plot3(x_vec,y_vec,z_vec,'.')
p_new = plot3(x_vec,y_vec,z_vec,'*')
axis([-10 10 -10 10 -10 10])
%view(20,14)
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
    [x,y,z,err] = parseData(data);
    if err 
        break
    end
    set(p,'XData',x_vec,'YData',y_vec,'ZData',z_vec)
    set(p_new,'XData',x(end),'YData',y(end),'ZData',z(end))
    x_vec = [x_vec,x];
    y_vec = [y_vec,y];
    z_vec = [z_vec,z];
    pause(0.1)
    clc
    disp("Samples:")
    disp(length(x_vec))
end
%hold off
disp("Port closed")
fclose(t)

function [x,y,z,err] = parseData(data)
    err = false;
    if contains(data,'q')
        err = true;
        x = [];
        y = [];
        z = [];
        return
    end
    data = sscanf(data,'%f',[3 inf])';
    x = data(:,1)';
    y = data(:,2)';
    z = data(:,3)';
end