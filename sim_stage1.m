clear;
clc;
%가로 error_pixel 세로 error_pixel 나누기
center_pts = [480,200];
center_a = [480, 260];
drone = ryze();
cam = camera(drone);
takeoff(drone);
%멀때  error_pixel
Err_pixel = 20;



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

%% 두번째 calibration 후에 조금씩 전진하면서 계속 위치를 조정하는 알고리즘
while 1
    x = input('f == 전진 t == 스탑','s');
    disp(newline);
    if x == 'f'
        moveforward(drone, 'Distance', 0.5);
        [frame , ~] = snapshot(cam);
        hsv = rgb2hsv(frame);
        h = hsv(:,:,1);
        s = hsv(:,:,2);
        v = hsv(:,:,3);

        % 2-2) 파란색 마스크
        blue_mask = (h > 0.55) & (h < 0.75) & (s > 0.4) & (v > 0.2);
        blue_mask_clean = bwareafilt(blue_mask, 1);

        % 2-3) 파란색 천 인식
        props_blue = regionprops(blue_mask_clean, 'BoundingBox');



        if isempty(props_blue)
            warning("파란 영역도 못 찾았습니다. 멈춥니다.");
            break; %파란 영역 인식 못했으면 알고리즘 생각
        end
        bbox = props_blue(1).BoundingBox;

        %2-4) 파란색 천의 중심, 너비, 높이 를 통해 threshold를 넘어갔는지 판단 그리고 좌우정렬 알고리즘
        imshow(frame); hold on
        rectangle('Position', bbox, 'EdgeColor', 'b', 'LineWidth', 1);
        hold off


        if (bbox(3) > 700 && bbox(4) > 500)
            centers = [ bbox(1) + bbox(3)/2,  bbox(2) + bbox(4)/2 ];
            dis = centers - center_a;
            if abs(dis(1)) < Err_pixel && abs(dis(2)) < Err_pixel
                disp("좌우 정렬 완료 — 루프 탈출");
                break;
            end
            if dis(1) > Err_pixel_1
                moveright(drone,'Distance',0.2);
                fprintf("move right\n");
            elseif dis(1) < -Err_pixel_1
                moveleft(drone,'Distance',0.2);
                fprintf("move left\n");
            end
            if dis(2) > Err_pixel_1
                movedown(drone,'Distance',0.2);
                fprintf("movedown\n");
            elseif dis(2) < -Err_pixel_1
                moveup(drone,'Distance',0.2);
                fprintf("moveup\n");
            end
        end
    elseif x == 't'
        land(drone)
        break;
    end
end

%% 원통과 일단 시켜보기
while 1
    x = input('f == 전진 t == 스탑','s');
    if x == 'f'
        moveforward(drone,'Distance', 0.5,'Speed',1);
    elseif x == 't'
        land(drone)
        break;
    end
end


