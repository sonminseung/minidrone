clear;
clc;
%가로 error_pixel 세로 error_pixel 나누기
center_pts = [480,200];
center_a = [480, 260];
drone = ryze();
cam = camera(drone);
takeoff(drone);


Err_pixel = 30;
Err_pixel_near = 60;
circleflag = 0;



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
    stats = regionprops(binary_res, 'Centroid', 'Circularity', 'Area','MajorAxisLength','MinorAxisLength');
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
        largeMask  = areas >= 700;          % logical mask
        idxLarge   = idxCircle(largeMask);   % stats 인덱스
        areasLarge = areas(largeMask);       % 필터된 면적값

        if isempty(idxLarge)
            warning("면적 ≥700인 원형 후보가 없습니다.");

            % (여기서 대체 로직을 넣어도 좋습니다)
        end

        % 필터된 후보 중 가장 큰 녀석 찾기
        [~, relMax] = max(areasLarge);
        idx         = idxLarge(relMax);
        centers = stats(idx).Centroid;
        disp("원 중심 사용")
    end

    imshow(frame); hold on
    plot(centers(1), centers(2), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
    hold off
    pause(3);
    % 1-4) 화면 중심과의 오차 계산(가까운 거리에서)
    dis = centers - center_pts;
    if width == 960
        % 1-5) 좌우 회전
        if abs(dis(1)) < Err_pixel_near && abs(dis(2)) < Err_pixel_near
            disp("캘리브레이션 완료 — 루프 탈출");
            Major = stats(idx).MajorAxisLength;
            Minor = stats(idx).MinorAxisLength;

            Avg_diameter = (Major+Minor) / 2;
            circleflag = 1;
            break;
        end
        if dis(1) > Err_pixel_near
            moveright(drone,'Distance',0.2);
            fprintf("move right\n");
        elseif dis(1) < -Err_pixel_near
            moveleft(drone,'Distance',0.2);
            fprintf("move left\n");
        end

        % 1-6) 상하 이동
        if dis(2) > Err_pixel_near
            movedown(drone,'Distance',0.2);
            fprintf("move down\n");
        elseif dis(2) < -Err_pixel_near
            moveup(drone,'Distance',0.2);
            fprintf("move up\n");
        end
    else
        if abs(dis(1)) < Err_pixel && abs(dis(2)) < Err_pixel
            disp("캘리브레이션 완료 — 루프 탈출");
            Major = stats(idx).MajorAxisLength;
            Minor = stats(idx).MinorAxisLength;

            Avg_diameter = (Major+Minor) / 2;
            circleflag = 1;
            break;
            % 1-5) 좌우 회전
        elseif dis(1) > Err_pixel
            moveright(drone,'Distance',0.2);
            fprintf("move right\n");
        elseif dis(1) < -Err_pixelx`
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
end
%% 원통과 일단 시켜보기
x = input('f == 전진 t == 스탑','s');
if x == 'f'
    if circleflag == 1
        dist = 67.63*exp(-0.02768*Avg_diameter)+3.229*exp(-0.00246*Avg_diameter);
        dist = round(dist,1);
        fprintf("%f", dist);
        dist = dist+0.4;
        moveforward(drone, 'Distance',dist, 'Speed',1);
    else
        dist = 6.376*exp(-0.001567*width);
        dist = round(dist,1);
        fprintf("%f", dist);
        dist = dist+0.4;
        moveforward(drone, 'Distance',dist, 'Speed',1);
    end

elseif x == 't'
    land(drone)
end
pause(3);
land(drone);

