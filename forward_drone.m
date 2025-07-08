clear;
clc;

center_pts = [480,200];
drone = ryze();
cam = camera(drone);
takeoff(drone);
Err_pixel = 50;
while 1
    x = input('f == 전진 t == 스탑','s');
    if x == 'f'
        moveforward(drone,'Distance', 1,'Speed',1);
    elseif x == 't'
        land(drone)
        break;
    end
end