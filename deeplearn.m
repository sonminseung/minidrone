%% 1. 데이터 정의
x_data = single([ ...
    933, 926, 942, 930, 896, ...
    898, 892, 894, 905, 900, ...
    877, 890, 892, 890, 884, ...
    682, 686, 686, 682, 684, ...
    664, 668, 666, 668, 664, ...
    618, 616, 616, 620, 616, ...
    564, 564, 568, 570, 573, ...
    530, 531, 536, 532, 530, ...
    512, 513, 518, 520, 522 ]);

y_data = single([ ...
    1.4, 1.4, 1.4, 1.4, 1.4, ...
    1.6, 1.6, 1.6, 1.6, 1.6, ...
    1.8, 1.8, 1.8, 1.8, 1.8, ...
    2.0, 2.0, 2.0, 2.0, 2.0, ...
    2.2, 2.2, 2.2, 2.2, 2.2, ...
    2.4, 2.4, 2.4, 2.4, 2.4, ...
    2.6, 2.6, 2.6, 2.6, 2.6, ...
    2.8, 2.8, 2.8, 2.8, 2.8, ...
    3.0, 3.0, 3.0, 3.0, 3.0 ]);

%% 2. 정규화 (Min-Max Scaling)
x_min = min(x_data);
x_max = max(x_data);
x_norm = (x_data - x_min) / (x_max - x_min);

x_all = x_norm(:);  % (N×1) 숫자형 배열
y_all = y_data(:);  % (N×1) 숫자형 배열

%% 3. 학습/테스트 데이터 분할
cv = cvpartition(length(x_all), 'HoldOut', 0.2);
xTrain = x_all(training(cv));
yTrain = y_all(training(cv));
xTest = x_all(test(cv));
yTest = y_all(test(cv));

%% 4. 회귀용 신경망 구성 (Dense NN)
layers = [
    featureInputLayer(1)
    fullyConnectedLayer(16)
    reluLayer
    fullyConnectedLayer(8)
    reluLayer
    fullyConnectedLayer(1)
    regressionLayer
];

options = trainingOptions('adam', ...
    'MaxEpochs', 500, ...
    'MiniBatchSize', 8, ...
    'Shuffle', 'every-epoch', ...
    'ValidationData', {xTest, yTest}, ...
    'ValidationPatience', 10, ...
    'Verbose', false, ...
    'Plots', 'training-progress');

%% 5. 학습 실행
net = trainNetwork(xTrain, yTrain, layers, options);

%% 6. 예측 및 평가
yPred = predict(net, xTest);

% RMSE, MAE 계산
rmse = sqrt(mean((yTest - yPred).^2));
mae = mean(abs(yTest - yPred));

fprintf('📉 RMSE: %.4f m\n', rmse);
fprintf('📏 MAE: %.4f m\n', mae);

%% 7. 예측 결과 시각화
figure;
scatter(yTest, yPred, 'g', 'filled');
hold on;
plot([min(yTest), max(yTest)], [min(yTest), max(yTest)], 'r--');
xlabel('실제 거리 (m)');
ylabel('예측 거리 (m)');
title('Dense NN 기반 거리 예측 결과');
grid on;
legend('예측값', 'y = x');

%% 8. 새로운 값 예측 함수 (예: 570 입력)
x_input = 570;
x_input_norm = (x_input - x_min) / (x_max - x_min);  % 정규화
y_output = predict(net, x_input_norm);
fprintf('\n[예측 결과] 입력 X = %.0f → 예측 Y = %.3f m\n', x_input, y_output);
