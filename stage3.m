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

%1. 맨처음 calibration
for i = 1:3

    [frame , ~] = snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    v = hsv(:,:,3);

    binary_res = (0.55 < h) & (h < 0.75) & (0.5 < s) ;
    binary_res = xor(binary_res, 1);

    stats = regionprops(binary_res, 'Centroid', 'Circularity', 'Area');
    if isempty(stats)
        warning('타겟 미검출');
        moveup(drone,'Distance',0.2);
        continue;
    end
    circVals = [stats.Circularity];

    idxCircle = find((circVals > 0.7));
    areas = [stats(idxCircle).Area];
    Large_area = areas > 6000;
    [maxArea, relMax] = max(Large_area);
    if isempty(maxArea)
        warning("원이 없습니다.");
        %여기서 턴
        continue;
    end

    idx = idxCircle(relMax);

    centers = stats(idx).Centroid;
    imshow(frame); hold on
    plot(centers(1), centers(2), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
    hold off
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
end

% 두번째 calibration 후에 조금씩 전진하면서 계속 위치를 조정하는 알고리즘

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







