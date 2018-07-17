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
p = plot(z_vec,y_vec,'.')
p_new = plot(z_vec,y_vec,'*')
while length(x_vec) <100 
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
    [x,y,z] = parseData(data);
    set(p,'XData',z_vec,'YData',y_vec)
    set(p_new,'XData',z(end),'YData',y(end))
    x_vec = [x_vec,x];
    y_vec = [y_vec,y];
    z_vec = [z_vec,z];
    pause(0.2)
    clc
    disp("Samples:")
    disp(length(x_vec))
end
%hold off
disp("Port closed")
fclose(t)

function [x,y,z] = parseData(data)
    data = sscanf(data,'%f',[3 inf])';
    x = data(:,1)';
    y = data(:,2)';
    z = data(:,3)';
end