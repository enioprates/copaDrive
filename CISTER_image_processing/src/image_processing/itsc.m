%ITSC 2020
%%
%% ajustando formato de dados
format long;
format compact;
%close all;
%clear;
%clc;
%% Loading Data
%TV
filename = 'log_position_SV_car1.csv';
data_tv = load(filename);

n_cars = 2;

%SV 2
filename = 'log_position_SV_car2.csv';
data_sv2 = load(filename);

%SV 3
filename = 'log_position_SV_car3.csv';
data_sv3 = load(filename);

%SV 4
filename = 'log_position_SV_car4.csv';
data_sv4 = load(filename);


%SV 5

filename = 'log_position_SV_car5.csv';
data_sv5 = load(filename);

%SV 6
filename = 'log_position_SV_car6.csv';
data_sv6 = load(filename);

%SV 7
filename = 'log_position_SV_car7.csv';
data_sv7 = load(filename);

%SV 8
filename = 'log_position_SV_car8.csv';
data_sv8 = load(filename);

%SV 9
filename = 'log_position_SV_car9.csv';
data_sv9 = load(filename);

%SV 10
filename = 'log_position_SV_car10.csv';
data_sv10 = load(filename);

% %SV 11
% filename = 'log_position_SV_car11.csv';
% data_sv11 = load(filename);

text = 'LOAD Graph!'
%% 
%1 - time;
%2 - count_SV;
%3 - latitude_SV;
%4 - longitude_SV;
%5 - speed_SV;
%6 - speed km;
%7 - heading_SV;
%8 - PID Error;
%9 - Theta Error;
%10 - lap_SV



%% trajectory 
%TV
plot(data_tv(:,3)',data_tv(:,4)')
hold on
plot(data_sv2(:,3)',data_sv2(:,4)')
plot(data_sv3(:,3)',data_sv3(:,4)')
plot(data_sv4(:,3)',data_sv4(:,4)')
plot(data_sv5(:,3)',data_sv5(:,4)')
plot(data_sv6(:,3)',data_sv6(:,4)')
plot(data_sv7(:,3)',data_sv7(:,4)')
plot(data_sv8(:,3)',data_sv8(:,4)')
plot(data_sv9(:,3)',data_sv9(:,4)')
plot(data_sv10(:,3)',data_sv10(:,4)')
% plot(data_sv11(:,3)',data_sv11(:,4)')


title('Trajectory Comparison');
xlabel ('Longitude (m)');
ylabel('Latitude (m)');
%yticks(3:0.1:4)
%legend('TV');
% legend('TV','SV2');
% legend('TV','SV2','SV3');
% legend('TV','SV2','SV3','SV4');
%legend('TV','SV2','SV3','SV4','SV5');
% legend('TV','SV2','SV3','SV4','SV5','SV6');
% legend('TV','SV2','SV3','SV4','SV5','SV6','SV7');
% legend('TV','SV2','SV3','SV4','SV5','SV6','SV7','SV8');
legend('TV','SV2','SV3','SV4','SV5','SV6','SV7','SV8','SV9','SV10','SV11');
hold off


%% PID Error
used_coluumn = 8;
%Zerando a coluna do tempo
data_pid = data_sv2;                                %atribuindo a coluna do tempo desejada
tam = size(data_pid(:,1),1);
for i=2:tam,                                        %de 1 ate o tamanho da coluna do tempo
    data_pid(i,1)=data_pid(i,1)-data_pid(1,1);      %inicia o tempo em zero
end
data_pid(1,1)= 0;

%SV2
tam = size(data_sv2(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv2(1:end,used_coluumn)', '-')
hold on
% %SV3
% tam = size(data_sv3(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv3(:,used_coluumn)')
% %SV4
% tam = size(data_sv4(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv4(:,used_coluumn)')
% %SV5
tam = size(data_sv5(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv5(:,used_coluumn)')
% %SV6
% tam = size(data_sv6(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv6(:,used_coluumn)')
% %SV7
% tam = size(data_sv7(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv7(:,used_coluumn)')
% %SV8
% tam = size(data_sv8(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv8(:,used_coluumn)')
% %SV9
% tam = size(data_sv9(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv9(:,used_coluumn)')
%SV10
tam = size(data_sv10(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv10(:,used_coluumn)'+1)
% %SV11
% tam = size(data_sv11(:,used_coluumn),1);
% plot(data_pid(750:tam,1)',data_sv11(750:end,used_coluumn)')

title('Distance Error');
xlabel ('Time (s)');
ylabel('Error (m)');
legend('SV2','SV3','SV4','SV5','SV6','SV7','SV8','SV9','SV10','SV11');
%ylim([-1 10]);
%xticks(0:10:500);
hold off

%% Box Plot - PID ERROR

used_coluumn = 8;
%Zerando a coluna do tempo
data_pid = data_sv2;                                %atribuindo a coluna do tempo desejada
tam = size(data_pid(:,1),1);
for i=2:tam,                                        %de 1 ate o tamanho da coluna do tempo
    data_pid(i,1)=data_pid(i,1)-data_pid(1,1);      %inicia o tempo em zero
end
data_pid(1,1)= 0;

pid_err_vec = zeros(10,1);

%SV2
k1 = data_sv2(1:end,used_coluumn);
%SV3
k2 = data_sv3(1:end,used_coluumn);
%SV4
k3 = data_sv4(1:end,used_coluumn);
%SV5
k4 = data_sv5(1:end,used_coluumn);
%SV6
k5 = data_sv6(1:end,used_coluumn);
%SV7
k6 = data_sv7(1:end,used_coluumn);
%SV8
k7 = data_sv8(1:end,used_coluumn);
%SV9
k8 = data_sv9(1:end,used_coluumn);
%SV10
k9 = data_sv10(1:end,used_coluumn);
%SV11
k10 = data_sv11(1:end,used_coluumn);

k = [k1 k2 k3 k4 k5 k6 k7 k8 k9 k10];

%k = [k1 k2 k3 k4];

labels = {'SV1-SV2','SV2-SV3', 'SV3-SV4','SV4-SV5', 'SV5-SV6','SV6-SV7', 'SV7-SV8','SV8-SV9', 'SV9-SV10','SV10-SV11'};
%labels = {'SV1-SV2','SV2-SV3','SV3-SV4','SV4-SV5'};

boxplot(k,'Whisker',20, 'Labels', labels)
hold on

title('Distance Error');
xlabel ('Vehicle (s)');
ylabel('Error (m)');
xtickangle(45);
%legend('SV2','SV3','SV4','SV5','SV6','SV7','SV8','SV9','SV10','SV11');
%legend('SV2');
ylim([-5 10]);
%xticks(0:10:500);
hold off



%% Theta Error (graus)
used_coluumn = 9;
%Zerando a coluna do tempo
data_pid = data_sv2;                                %atribuindo a coluna do tempo desejada
tam = size(data_pid(:,1),1);
for i=2:tam,                                        %de 1 ate o tamanho da coluna do tempo
    data_pid(i,1)=data_pid(i,1)-data_pid(1,1);      %inicia o tempo em zero
end
data_pid(1,1)= 0;
%SV2
tam = size(data_sv2(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv2(:,used_coluumn)'*(180/pi()))
hold on
%SV3
tam = size(data_sv3(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv3(:,used_coluumn)'*(180/pi()))
%SV4
tam = size(data_sv4(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv4(:,used_coluumn)'*(180/pi()))
%SV5
tam = size(data_sv5(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv5(:,used_coluumn)'*(180/pi()))
%SV6
tam = size(data_sv6(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv6(:,used_coluumn)'*(180/pi()))
%SV7
tam = size(data_sv7(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv7(:,used_coluumn)'*(180/pi()))
%SV8
tam = size(data_sv8(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv8(:,used_coluumn)'*(180/pi()))
%SV9
tam = size(data_sv9(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv9(:,used_coluumn)'*(180/pi()))
%SV10
tam = size(data_sv10(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv10(:,used_coluumn)'*(180/pi()))
%SV11
tam = size(data_sv11(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv11(:,used_coluumn)'*(180/pi()))

title('Theta Error');
xlabel ('Time (s)');
ylabel('Error (GRAUS)');
legend('SV2','SV3','SV4','SV5','SV6','SV7','SV8','SV9','SV10','SV11');

hold off

%% Theta Error (RAD)
used_coluumn = 9;
%Zerando a coluna do tempo
data_pid = data_sv2;                                %atribuindo a coluna do tempo desejada
tam = size(data_pid(:,1),1);
for i=2:tam,                                        %de 1 ate o tamanho da coluna do tempo
    data_pid(i,1)=data_pid(i,1)-data_pid(1,1);      %inicia o tempo em zero
end
data_pid(1,1)= 0;
%SV2
tam = size(data_sv2(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv2(:,used_coluumn)')
hold on
%SV3
tam = size(data_sv3(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv3(:,used_coluumn)')
%SV4
tam = size(data_sv4(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv4(:,used_coluumn)')
%SV5
tam = size(data_sv5(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv5(:,used_coluumn)')
%SV6
tam = size(data_sv6(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv6(:,used_coluumn)')
%SV7
tam = size(data_sv7(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv7(:,used_coluumn)')
%SV8
tam = size(data_sv8(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv8(:,used_coluumn)')
%SV9
tam = size(data_sv9(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv9(:,used_coluumn)')
%SV10
tam = size(data_sv10(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv10(:,used_coluumn)')
%SV11
tam = size(data_sv11(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv11(:,used_coluumn)')


title('Theta Error');
xlabel ('Time (s)');
ylabel('Error (rad)');
legend('SV2','SV3','SV4','SV5','SV6','SV7','SV8','SV9','SV10','SV11');

hold off
%% Theta Comparison
used_coluumn = 7;
%Zerando a coluna do tempo
data_pid = data_sv2;                                %atribuindo a coluna do tempo desejada
tam = size(data_pid(:,1),1);
for i=2:tam,                                        %de 1 ate o tamanho da coluna do tempo
    data_pid(i,1)=data_pid(i,1)-data_pid(1,1);      %inicia o tempo em zero
end
data_pid(1,1)= 0;
%TV
tam = size(data_tv(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_tv(:,used_coluumn)')
hold on
%SV2
tam = size(data_sv2(:,used_coluumn),1);
plot(data_pid(1:tam,1)',data_sv2(:,used_coluumn)')
% %SV3
% tam = size(data_sv3(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv3(:,used_coluumn)')
% %SV4
% tam = size(data_sv4(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv4(:,used_coluumn)')
% %SV5
% tam = size(data_sv5(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv5(:,used_coluumn)')
% %SV6
% tam = size(data_sv6(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv6(:,used_coluumn)')
% %SV7
% tam = size(data_sv7(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv7(:,used_coluumn)')
% %SV8
% tam = size(data_sv8(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv8(:,used_coluumn)')
% %SV9
% tam = size(data_sv9(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv9(:,used_coluumn)')
% %SV10
% tam = size(data_sv10(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv10(:,used_coluumn)')
% %SV11
% tam = size(data_sv11(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv11(:,used_coluumn)')


title('Theta Comparison');
xlabel ('Time (s)');
ylabel('Theta (rad)');
legend('TV','SV2','SV3','SV4','SV5','SV6','SV7','SV8','SV9','SV10','SV11');
hold off

%% Theta Error (new)
used_coluumn = 7;
%Zerando a coluna do tempo
data_pid = data_sv2;                                %atribuindo a coluna do tempo desejada
tam = size(data_pid(:,1),1);
for i=2:tam,                                        %de 1 ate o tamanho da coluna do tempo
    data_pid(i,1)=data_pid(i,1)-data_pid(1,1);      %inicia o tempo em zero
end
data_pid(1,1)= 0;
%TV - SV2
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_tv(1:tam,used_coluumn)')-abs(data_sv2(1:tam,used_coluumn)'))
hold on

%SV2 - SV3
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_sv2(1:tam,used_coluumn)')-abs(data_sv3(1:tam,used_coluumn)'))

%SV3 - SV4
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_sv3(1:tam,used_coluumn)')-abs(data_sv4(1:tam,used_coluumn)'))

%SV4 - SV5
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_sv4(1:tam,used_coluumn)')-abs(data_sv5(1:tam,used_coluumn)'))

%SV5 - SV6
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_sv5(1:tam,used_coluumn)')-abs(data_sv6(1:tam,used_coluumn)'))

%SV6 - SV7
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_sv6(1:tam,used_coluumn)')-abs(data_sv7(1:tam,used_coluumn)'))

%SV7 - SV8
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_sv7(1:tam,used_coluumn)')-abs(data_sv8(1:tam,used_coluumn)'))

%SV8 - SV9
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_sv8(1:tam,used_coluumn)')-abs(data_sv9(1:tam,used_coluumn)'))

%SV9 - SV10
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_sv9(1:tam,used_coluumn)')-abs(data_sv10(1:tam,used_coluumn)'))

%SV10 - SV11
tam = size(data_tv(:,used_coluumn),1)-1;
plot(data_pid(1:tam,1)',abs(data_sv10(1:tam,used_coluumn)')-abs(data_sv11(1:tam,used_coluumn)'))


title('Theta Error');
xlabel ('Time (s)');
ylabel('Theta (rad)');
legend('SV2','SV3','SV4','SV5','SV6','SV7','SV8','SV9','SV10','SV11');
hold off

%% Calcula Distancia
%used columns
lat = 3;
long = 4;

%obstacle vector
v1 = [250.63,43.61];
v2 = [280.20, 48.18];
v3 = [307.18, 43.76];
v4 = [335.75, 47.40];
v5 = [364.76, 43.82];
v6 = [395.38, 47.71];
v7 = [447.76, 102.16];
v8 = [340.41, 160.65];
v9 = [333.56, 160.67];
v10 = [326.31, 160.78];
v11 = [54.30, 283.50];
v12 = [50.76, 253.01];
v13 = [54.60, 223.71];
v14 = [50.86, 195.07];
v15 = [50.80, 188.39];
pos = [v1 ; v2 ; v3; v4;v5;v6;v7;v8;v9;v10;v11;v12;v13;v14;v15];

tam = size (pos, 1);
dist = zeros(tam,n_cars);

for i = 1:tam,

    %TV
    dist(i,1) = min(sqrt((data_tv(:,lat)-pos(i,1)).^2 + (data_tv(:,long)-pos(i,2)).^2))-2;
    %SV2
    dist(i,2) = min(sqrt((data_sv2(:,lat)-pos(i,1)).^2 + (data_sv2(:,long)-pos(i,2)).^2))-2;
    %SV3
    dist(i,3) = min(sqrt((data_sv3(:,lat)-pos(i,1)).^2 + (data_sv3(:,long)-pos(i,2)).^2))-2;
    %SV4
    dist(i,4) = min(sqrt((data_sv4(:,lat)-pos(i,1)).^2 + (data_sv4(:,long)-pos(i,2)).^2))-2;
    %SV5
    dist(i,5) = min(sqrt((data_sv5(:,lat)-pos(i,1)).^2 + (data_sv5(:,long)-pos(i,2)).^2))-2;
    %SV6
    dist(i,6) = min(sqrt((data_sv6(:,lat)-pos(i,1)).^2 + (data_sv6(:,long)-pos(i,2)).^2))-2;
    %SV7
    dist(i,7) = min(sqrt((data_sv7(:,lat)-pos(i,1)).^2 + (data_sv7(:,long)-pos(i,2)).^2))-2;
    %SV8
    dist(i,8) = min(sqrt((data_sv8(:,lat)-pos(i,1)).^2 + (data_sv8(:,long)-pos(i,2)).^2))-2;
    %SV9
    dist(i,9) = min(sqrt((data_sv9(:,lat)-pos(i,1)).^2 + (data_sv9(:,long)-pos(i,2)).^2))-2;
    %SV10
    dist(i,10) = min(sqrt((data_sv10(:,lat)-pos(i,1)).^2 + (data_sv10(:,long)-pos(i,2)).^2))-2;
    %SV11
    dist(i,11) = min(sqrt((data_sv11(:,lat)-pos(i,1)).^2 + (data_sv11(:,long)-pos(i,2)).^2))-2;    
end

%for i=1:
max_dist = max(dist);
min_dist = min(dist);
avg_dist = mean(dist);

geral = [min_dist; max_dist ; avg_dist];
bar (geral);

title('Obstacle Distance');
%xlabel ('Time (s)');
ylabel('Distance (m)');
legend('TV','SV2','SV3','SV4','SV5','SV6','SV7','SV8','SV9','SV10','SV11');

%% Stability
used_coluumn = 8;
%Zerando a coluna do tempo
norm_vec = zeros(10,1);

%SV2
norm_vec(1) = norm(data_sv2(750:end,used_coluumn),inf);
%SV3
norm_vec(2) = norm(data_sv3(750:end,used_coluumn),inf);
%SV4
norm_vec(3) = norm(data_sv4(750:end,used_coluumn),inf);
%SV5
norm_vec(4) = norm(data_sv5(750:end,used_coluumn),inf);
% %SV6
% norm_vec(5) = norm(data_sv6(750:end,used_coluumn),inf);
% %SV7
% norm_vec(6) = norm(data_sv7(750:end,used_coluumn),inf);
% %SV8
% norm_vec(7) = norm(data_sv8(750:end,used_coluumn),inf);
% %SV9
% norm_vec(8) = norm(data_sv9(750:end,used_coluumn),inf);
% %SV10
% norm_vec(9) = norm(data_sv10(750:end,used_coluumn),inf);
% %SV11
% norm_vec(10) = norm(data_sv11(750:end,used_coluumn),inf);

bar(norm_vec)

%% Distance
used_coluumn = 11;
%Zerando a coluna do tempo
data_pid = data_sv2;                                %atribuindo a coluna do tempo desejada
tam = size(data_pid(:,1),1);
for i=2:tam,                                        %de 1 ate o tamanho da coluna do tempo
    data_pid(i,1)=data_pid(i,1)-data_pid(1,1);      %inicia o tempo em zero
end
data_pid(1,1)= 0;

%SV2
tam = size(data_sv2(:,used_coluumn),1);
plot(data_pid(10:tam,1)',data_sv2(10:end,used_coluumn)', '-')
hold on
%SV3
tam = size(data_sv3(:,used_coluumn),1);
plot(data_pid(10:tam,1)',data_sv3(10:tam,used_coluumn)')
%SV4
tam = size(data_sv4(:,used_coluumn),1);
plot(data_pid(10:tam,1)',data_sv4(10:tam,used_coluumn)')
% %SV5
tam = size(data_sv5(:,used_coluumn),1);
plot(data_pid(10:tam,1)',data_sv5(10:tam,used_coluumn)')
% %SV6
% tam = size(data_sv6(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv6(:,used_coluumn)')
% %SV7
% tam = size(data_sv7(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv7(:,used_coluumn)')
% %SV8
% tam = size(data_sv8(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv8(:,used_coluumn)')
% %SV9
% tam = size(data_sv9(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv9(:,used_coluumn)')
% %SV10
% tam = size(data_sv10(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv10(:,used_coluumn)'+1)
% %SV11
% tam = size(data_sv11(:,used_coluumn),1);
% plot(data_pid(750:tam,1)',data_sv11(750:end,used_coluumn)')

title('Inter Vehicle Distance');
xlabel ('Time (s)');
ylabel('Distance (m)');
legend('SV1-SV2','SV2-SV3','SV3-SV4','SV4-SV5','SV6','SV7','SV8','SV9','SV10','SV11');
%ylim([-1 10]);
%xticks(0:10:500);
hold off

%% Box Plot - Distance

used_coluumn = 11;
%Zerando a coluna do tempo
data_pid = data_sv2;                                %atribuindo a coluna do tempo desejada
tam = size(data_pid(:,1),1);
for i=2:tam,                                        %de 1 ate o tamanho da coluna do tempo
    data_pid(i,1)=data_pid(i,1)-data_pid(1,1);      %inicia o tempo em zero
end
data_pid(1,1)= 0;

%SV2
tam = size(data_sv2(:,used_coluumn),1);
%boxplot(data_pid(1:tam,1)',data_sv2(1:end,used_coluumn)', '-')
boxplot(data_sv2(1:end,used_coluumn))
hold on
% %SV3
% tam = size(data_sv3(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv3(:,used_coluumn)')
% %SV4
% tam = size(data_sv4(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv4(:,used_coluumn)')
% % %SV5
% tam = size(data_sv5(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv5(:,used_coluumn)')
% %SV6
% tam = size(data_sv6(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv6(:,used_coluumn)')
% %SV7
% tam = size(data_sv7(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv7(:,used_coluumn)')
% %SV8
% tam = size(data_sv8(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv8(:,used_coluumn)')
% %SV9
% tam = size(data_sv9(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv9(:,used_coluumn)')
% %SV10
% tam = size(data_sv10(:,used_coluumn),1);
% plot(data_pid(1:tam,1)',data_sv10(:,used_coluumn)'+1)
% %SV11
% tam = size(data_sv11(:,used_coluumn),1);
% plot(data_pid(750:tam,1)',data_sv11(750:end,used_coluumn)')

title('Distance Error');
xlabel ('Vehicle (s)');
ylabel('Error (m)');
%legend('SV2','SV3','SV4','SV5','SV6','SV7','SV8','SV9','SV10','SV11');
%legend('SV2');
%ylim([-1 10]);
%xticks(0:10:500);
hold off

