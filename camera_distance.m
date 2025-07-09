clear;
clc;

real_Circle = 0.46;

center_pts = [480,180];
drone = ryze();
cam = camera(drone);
takeoff(drone);
Err_pixel = 30;
Err_pixel_1 = 50;
moveup(drone,'Distance',0.7,'Speed',1);
pause(3);


% h = 0.55, 0.65 사이
% s =  0.625, 0.775
% v = 0.325 , 0.475;

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
    % 1-5) 좌우 정렬
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
        fprintf("movedown\n");
    elseif dis(2) < -Err_pixel
        moveup(drone,'Distance',0.2);
        fprintf("moveup\n");
    end
end

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
stats = regionprops(binary_res, 'Centroid', 'Circularity', 'Area','MajorAxisLength','MinorAxisLength');

% --- 원이 보이면 원의 중심 사용 ---
circVals  = [stats.Circularity];
idxCircle = find(circVals > 0.7);

% 원형성 조건을 만족하는 객체들의 면적 구하기
areas     = [stats(idxCircle).Area];

% 면적이 1000 이상인 것만 선택
largeMask  = areas >= 1000;          % logical mask
idxLarge   = idxCircle(largeMask);   % stats 인덱스
areasLarge = areas(largeMask);       % 필터된 면적값

% 필터된 후보 중 가장 큰 녀석 찾기
[~, relMax] = max(areasLarge);
idx         = idxLarge(relMax);

centers = stats(idx).Centroid;
Major = stats(idx).MajorAxisLength;
Minor = stats(idx).MinorAxisLength;
diameter_pixel = (Major + Minor)/2;


real_distance = (real_circle*)/diameter_pixel;





