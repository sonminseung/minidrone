clear;
clc;

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



%%

%가로 error_pixel 세로 error_pixel 나누기
center_pts = [480,200];
center_a = [480, 260];
drone = ryze();
cam = camera(drone);
takeoff(drone);
%멀때  error_pixel
Err_pixel = 30;



%가까울때 error_pixel
Err_pixel_1 = 40;
moveup(drone,'Distance',0.7,'Speed',1);
pause(3);


%% 1. 맨처음 calibration
while 1
    % 1-1) 이미지 획득 & HSV 분리
    [frame , ~] = snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    v = hsv(:,:,3);

    % 1-2) 파란색 마스크
    blue_mask = (h > 0.55) & (h < 0.75) & (s > 0.4) & (v > 0.2);
    blue_mask_clean = bwareafilt(blue_mask, 1);

    % 1-3) 원 검출 시도
    binary_res = xor(blue_mask_clean, 1);
    stats = regionprops(binary_res, 'Centroid', 'Circularity', 'Area');
    props_blue =  regionprops(blue_mask_clean, 'BoundingBox');
    bbox = props_blue(1).BoundingBox;
    width = bbox(3);
    if isempty(stats)
        % --- 원이 안 보이면 bounding box 중심으로 대체 ---
        props_blue = regionprops(blue_mask_clean, 'BoundingBox');
        if isempty(props_blue)
            warning("파란 영역도 못 찾았습니다. 다시 시도합니다.");
            continue;   % 다음 반복으로
        end
        bbox = props_blue(1).BoundingBox;
        centers = [ bbox(1) + bbox(3)/2,  bbox(2) + bbox(4)/2 ];
        disp("원 대신 파란 박스 중심 사용")
    else
        % --- 원이 보이면 원의 중심 사용 ---
        circVals  = [stats.Circularity];
        idxCircle = find(circVals > 0.7);

        % 원형성 조건을 만족하는 객체들의 면적 구하기
        areas     = [stats(idxCircle).Area];

        % 면적이 1000 이상인 것만 선택
        largeMask  = areas >= 1000;          % logical mask
        idxLarge   = idxCircle(largeMask);   % stats 인덱스
        areasLarge = areas(largeMask);       % 필터된 면적값

        if isempty(idxLarge)
            warning("면적 ≥1000인 원형 후보가 없습니다.");
            % (여기서 대체 로직을 넣어도 좋습니다)
        else
            % 필터된 후보 중 가장 큰 녀석 찾기
            [~, relMax] = max(areasLarge);
            idx         = idxLarge(relMax);

            centers = stats(idx).Centroid;
            disp("원 중심 사용")
        end
    end

    imshow(frame); hold on
    plot(centers(1), centers(2), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
    hold off

    % 1-4) 화면 중심과의 오차 계산
    dis = centers - center_pts;

    if abs(dis(1)) < Err_pixel && abs(dis(2)) < Err_pixel
        disp("캘리브레이션 완료 — 루프 탈출");
        break;
    end
    % 1-5) 좌우 회전
    if dis(1) > Err_pixel
        moveright(drone,'Distance',0.2);
        fprintf("move right\n");
    elseif dis(1) < -Err_pixel
        moveleft(drone,'Distance',0.2);
        fprintf("move left\n");
    end

    % 1-6) 상하 이동
    if dis(2) > Err_pixel
        movedown(drone,'Distance',0.2);
        fprintf("move down\n");
    elseif dis(2) < -Err_pixel
        moveup(drone,'Distance',0.2);
        fprintf("move up\n");
    end
end

%% 원통과 일단 시켜보기
x = input('f == 전진 t == 스탑','s');
if x == 'f'
    %% 8. 새로운 값 예측 함수 (예: 570 입력)
    x_input = width;
    x_input_norm = (x_input - x_min) / (x_max - x_min);  % 정규화
    y_output = predict(net, x_input_norm);
    fprintf('\n[예측 결과] 입력 X = %.0f → 예측 Y = %.3f m\n', x_input, y_output);
elseif x == 't'
    land(drone)
end


