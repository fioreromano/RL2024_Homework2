% Percorso del file bag ROS 2
bag = ros2bag("C:\Users\hp\Desktop\ros2\subset_0.db3");

% Lista dei topic disponibili nel bag
topics = bag.AvailableTopics;
disp("Lista dei topic nel file bag:");
disp(topics);

% Specifica il topic di interesse
topic = "/effort_controller/commands";

% Leggi i messaggi dal topic
msgs = readMessages(select(bag, "Topic", topic));

% Verifica il numero di messaggi letti
disp("Numero di messaggi estratti:");
disp(length(msgs));

% Supponendo che ogni messaggio contenga un array di 6 elementi
torque_data = zeros(length(msgs), 7); % Prealloca la matrice (N x 6)
for k= 1 : 7 
for i = 1:length(msgs)
    torque_data(i,k) = msgs{i}.data(k);
end

end
% Mostra i primi 5 messaggi per verifica

% Supponiamo che la colonna del tempo si chiami 'Time'
time_data = bag.MessageList.Time; % Seleziona la colonna 'Time' della tabella
disp(time_data(1:5)); % Mostra i primi 5 valori del tempo

figure;
plot(time_data, torque_data, 'LineWidth', 1.5); % Plotta tutte le colonne
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6','Joint 7');
xlabel('Tempo [s]');
ylabel('Torque [Nm]');
title('Torque for Each Joint');
grid on;





