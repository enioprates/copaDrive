%Uporto
%%
%% ajustando formato de dados
format long;
format compact;
%close all;
%clear;
%clc;
%% Loading Data
% TV
filename = 'log_position_TV.csv';
data_tv = load(filename);

text = 'LOAD Graph!'
%% Adjusting Laps Size
text = 'Ajusting Laps'
n_laps = max(data_tv(:,8));         %salvando o numero de voltas percorridas

size_m = zeros(n_laps,1);           %criando vetor de zeros

for i=1:n_laps,                     %salvando o numero de ocorrencias em cada volta
   size_m(i,1) = sum(data_tv(:,8) == i); 
end

size_m_sum = zeros(n_laps,2);           %criando vetor de zeros
for i=1:n_laps,                     %salvando o numero de ocorrencias em cada volta
    if i==1
        size_m_sum(i,:) = [1,size_m(i,1)];
    else
        size_m_sum(i,:) = [size_m_sum(i-1,2)+1,size_m(i,1)+size_m_sum(i-1,2)];
    end
end

%% Criando nova Matriz

matriz_voltas = zeros(max(size_m),8,n_laps);           %criando vetor de zeros
%size(matriz_voltas);

for i=1:n_laps,                     %salvando o numero de ocorrencias em cada volta
    if i==1,
        text = 'aqui';
        size_m(i,1);
        matriz_voltas(1:size_m(i,1),:,i)= data_tv(1:size_m(i,1),:);
    else
        text = 'Outro';
        size_m(i,1);
        matriz_voltas(1:size_m(i,1),:,i)= data_tv(size_m_sum(i,1):size_m_sum(i,2),:);
        
    end
end


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
plot_vec = [7,8,10,11,12,16,20,22,23];
%plot_vec = [1,2,4,5,6,7,8,9,10];
for i=1:size(plot_vec(),2)
    k=plot_vec(1,i);
    plot(data_tv(size_m_sum(k,1):size_m_sum(k,2),3)',data_tv(size_m_sum(k,1):size_m_sum(k,2),4)')
    hold on
end




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
legend('C1','C2','C3','C4','C5','C6','C7','C8','C9','10','11','12','13','14','15','16','17','18','19','20','21','22','23');
hold off
