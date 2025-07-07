clear;
clc;

center_pts = [480,200];
drone = ryze();
cam = camera(drone);
takeoff(drone);
Err_pixel = 50;
moveup(drone,'Distance',1,'Speed',1);


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
    binary_res = xor(blue_mask_clean, 1);   % 혹시 원래 쓰시던 이진화 로직으로 바꿔주세요
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
        circVals   = [stats.Circularity];
        idxCircle  = find(circVals > 0.7);
        areas      = [stats(idxCircle).Area];
        [~,relMax] = max(areas);
        idx        = idxCircle(relMax);

        centers = stats(idx).Centroid;
        disp("원 중심 사용")
    end

    % 1-4) 화면 중심과의 오차 계산
    dis = centers - center_pts;

    % 1-4-1) if문 탈출 부분 
    if abs(dis(1)) > Err_pixel && abs(dis(2)) > Err_pixel
        break;
    end
    % 1-5) 좌우 회전
    if dis(1) > Err_pixel
        turn(drone,deg2rad(5));
        fprintf("turn right\n");
    elseif dis(1) < -Err_pixel
        turn(drone,deg2rad(-5));
        fprintf("turn left\n");
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
%% 두번째 calibration 후에 조금씩 전진하면서 계속 위치를 조정하는 알고리즘
%수정필요
while 1
    % 1. 이미지 불러오기
    [frame, ts] = snapshot(cam);
    img = frame;

    % 2. HSV 색 공간 변환
    hsv_img = rgb2hsv(img);
    h = hsv_img(:,:,1);
    s = hsv_img(:,:,2);
    v = hsv_img(:,:,3);

    % 3. 파란색 마스크 및 경계 박스
    blue_mask = (h > 0.55) & (h < 0.75) & (s > 0.4) & (v > 0.2);
    blue_mask_clean = bwareafilt(blue_mask, 1);

    props_blue = regionprops(blue_mask_clean, 'BoundingBox');
    bbox_blue  = props_blue.BoundingBox;

    crop_img_blue = imcrop(img, bbox_blue);
    if bbox_blue(3) > widthThreshold && bbox_blue(4) > heightThreshold
        % 파란 박스가 충분히 클 때 전진
        disp("파란색 천 앞까지 도착 완료")
        %여기 숫자 바꾸기 %,
        moveforward(drone, 'Distance',1.5 ,'Speed', 1);

        break
    end
    moveforward(drone, 'Distance', 0.5, 'Speed', 1); %조금씩 전진


    % 5. 파란 영역에서 원 검출
    gray_blue = rgb2gray(crop_img_blue);
    [centers, radii] = imfindcircles(gray_blue, [75 290], ...
        'Sensitivity', 0.95, 'EdgeThreshold', 0.108);

    y_offset = bbox_blue(1);
    x_offset = bbox_blue(2);

    dis = centers - center_pts;

    if dis(1) > Err_pixel
        moveright(drone,'Distance',0.2);
        fprintf("moveright\n");
    elseif dis(1) < -Err_pixel
        moveleft(drone, 'Distance',0.2)
        fprintf("moveleft\n");
    end

    if dis(2) > Err_pixel
        movedown(drone,'Distance',0.2);
        fprintf("movedown\n");
    elseif dis(2) < -Err_pixel
        moveup(drone,'Distance',0.2);
        fprintf("moveup\n");
    end

    % --- 시각화 ---
    subplot(2,2,1);
    imshow(img); hold on;
    rectangle('Position', bbox_blue, 'EdgeColor', 'b', 'LineWidth', 1);
    title('원본 이미지 + 파란 박스');
    plot(adjusted_centers(:,1), adjusted_centers(:,2), 'ro', ...
        'MarkerSize', 1, 'LineWidth', 1);
    hold off;

    subplot(2,2,2);
    imshow(crop_img_blue);
    title('파란색 영역');
end

turn(drone,deg2rad(90));





