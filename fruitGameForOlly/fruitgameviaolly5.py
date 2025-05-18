import pygame
from time import sleep
import threading
import random
import cv2
from picamera2 import Picamera2
from adafruit_servokit import ServoKit
import os

# -------------------- HARDWARE SETUP --------------------
kit = ServoKit(channels=16)
pygame.init()
screen = pygame.display.set_mode((640, 480)) 

pan_angle = 0.5  
tilt_angle = 0.5
kit.servo[0].angle = 180 - (pan_angle * 180)
kit.servo[1].angle = tilt_angle * 180

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

# ------------ CAMERA CONSTANTS ------------
FOV_H = 62  
FOV_V = 48
SCR_W, SCR_H = 640, 480
FRAME_RATE = 30
DT = 1.0 / FRAME_RATE

SPAWN_AZ_REL_RANGE = (-FOV_H / 2, FOV_H / 2)    
SPAWN_EL_REL_RANGE = (FOV_V / 2 + 10, FOV_V + 50) 

# -------------------- GAME OBJECTS --------------------
def load_fruit_images():
    asset_dir = "assets"
    fruit_images = {}

    # full fruit
    fruit_images["apple"] = cv2.imread(os.path.join(asset_dir, "apple.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["banana"] = cv2.imread(os.path.join(asset_dir, "banana.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["coconut"] = cv2.imread(os.path.join(asset_dir, "coconut.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["orange"] = cv2.imread(os.path.join(asset_dir, "orange.png"), cv2.IMREAD_UNCHANGED)

    # half fruit
    fruit_images["apple_half_1"] = cv2.imread(os.path.join(asset_dir, "apple_half_1.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["apple_half_2"] = cv2.imread(os.path.join(asset_dir, "apple_half_2.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["banana_half_1"] = cv2.imread(os.path.join(asset_dir, "banana_half_1.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["banana_half_2"] = cv2.imread(os.path.join(asset_dir, "banana_half_2.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["coconut_half_1"] = cv2.imread(os.path.join(asset_dir, "coconut_half_1.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["coconut_half_2"] = cv2.imread(os.path.join(asset_dir, "coconut_half_2.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["orange_half_1"] = cv2.imread(os.path.join(asset_dir, "orange_half_1.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["orange_half_2"] = cv2.imread(os.path.join(asset_dir, "orange_half_2.png"), cv2.IMREAD_UNCHANGED)

    # small fruit
    fruit_images["apple_small"] = cv2.imread(os.path.join(asset_dir, "apple_small.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["banana_small"] = cv2.imread(os.path.join(asset_dir, "banana_small.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["coconut_small"] = cv2.imread(os.path.join(asset_dir, "coconut_small.png"), cv2.IMREAD_UNCHANGED)
    fruit_images["orange_small"] = cv2.imread(os.path.join(asset_dir, "orange_small.png"), cv2.IMREAD_UNCHANGED)

    # Load explosion
    fruit_images["explosion"] = cv2.imread(os.path.join(asset_dir, "explosion.png"), cv2.IMREAD_UNCHANGED)

    return fruit_images

fruit_images = load_fruit_images()


class Fruit:
    def __init__(self, cam_pan_deg, fruit_images):
        self.fruit_type = random.choice(["apple", "banana", "coconut", "orange"])
        self.spawn_az_rel = random.uniform(SPAWN_AZ_REL_RANGE[0], SPAWN_AZ_REL_RANGE[1])
        self.spawn_el_rel = random.uniform(SPAWN_EL_REL_RANGE[0], SPAWN_EL_REL_RANGE[1])

        self.az = cam_pan_deg + self.spawn_az_rel
        self.el = self.spawn_el_rel

        direction = 1 if self.spawn_az_rel < 0 else -1
        self.v_az = random.uniform(10 * direction, 20 * direction)
        
        self.scored = False
        self.gravity = 120
        self.alive = True
        self.sliced = False
        self.slice_time = None
        self.fruit_images = fruit_images
        
        if random.random() < 0.5:  
            self.dist = random.uniform(1.0, 2.0)  
        else:
            self.dist = random.uniform(2.5, 4.0)  
            
        base_v_el = random.uniform(-80, -100)
        self.v_el = base_v_el / self.dist

    def update(self, dt):
        if not self.alive:
            return
        self.az += self.v_az * dt
        self.el += self.v_el * dt + 0.5 * self.gravity * dt * dt
        self.v_el += self.gravity * dt

        if self.el < -FOV_V:
            self.alive = False

        if self.sliced and self.slice_time and pygame.time.get_ticks() - self.slice_time > 500:
            self.alive = False

    def project(self, cam_pan_deg, cam_tilt_deg):
        rel_az = self.az - cam_pan_deg
        rel_el = self.el - cam_tilt_deg

        if abs(rel_az) > FOV_H / 2 or abs(rel_el) > FOV_V / 2:
            return None

        x = int((rel_az / (FOV_H / 2)) * (SCR_W / 2) + SCR_W / 2)
        y = int((rel_el / (FOV_V / 2)) * (SCR_H / 2) + SCR_H / 2)
        r = int(30 / self.dist)

        return x, y, r

    def draw_on_frame(self, frame, cam_pan_deg, cam_tilt_deg):
        proj = self.project(cam_pan_deg, cam_tilt_deg)
        if proj and self.alive:
            x, y, r = proj

            img = self.fruit_images[self.fruit_type]
            scale = 1 / self.dist
            h, w = img.shape[:2]
            scaled_w = int(w * scale)
            scaled_h = int(h * scale)

            resized_img = cv2.resize(img, (scaled_w, scaled_h), interpolation=cv2.INTER_AREA)

            x1 = x - scaled_w // 2
            y1 = y - scaled_h // 2

            try:
                
                for c in range(3):
                    frame[y1:y1+scaled_h, x1:x1+scaled_w, c] = (
                        resized_img[:, :, c] * (resized_img[:, :, 3] / 255.0) +
                        frame[y1:y1+scaled_h, x1:x1+scaled_w, c] * (1 - resized_img[:, :, 3] / 255.0)
                    )

                if self.sliced:
                    expl = cv2.resize(
                        self.fruit_images["explosion"],
                        (scaled_w, scaled_h),
                        interpolation=cv2.INTER_AREA
                    )
                    for c in range(3):
                        frame[y1:y1+scaled_h, x1:x1+scaled_w, c] = (
                            expl[:, :, c] * (expl[:, :, 3] / 255.0) +
                            frame[y1:y1+scaled_h, x1:x1+scaled_w, c] * (1 - expl[:, :, 3] / 255.0)
                        )
            except Exception as e:
                print(f"Error drawing fruit: {e}")

    def check_cut(self, cam_pan_deg, cam_tilt_deg):
        proj = self.project(cam_pan_deg, cam_tilt_deg)
        if not proj or not self.alive:
            return False
        x, y, r = proj
        laser_x, laser_y = SCR_W // 2, SCR_H // 2

        img = self.fruit_images[self.fruit_type]
        scale = 1 / self.dist
        h, w = img.shape[:2]
        scaled_w = int(w * scale)
        scaled_h = int(h * scale)

        if (
            laser_x >= x - scaled_w // 2 and
            laser_x <= x + scaled_w // 2 and
            laser_y >= y - scaled_h // 2 and
            laser_y <= y + scaled_h // 2
        ):
            if not self.scored:
                self.sliced = True
                self.slice_time = pygame.time.get_ticks()
                self.scored = True  # Only allow scoring once
                print("Fruit cut!")
                return True
        return False


fruits = []
spawn_timer = 0
score = 0
font = cv2.FONT_HERSHEY_SIMPLEX

# -------------------- SERVO CONTROL THREAD --------------------
def servo_control_thread():
    global pan_angle, tilt_angle
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            elif event.type == pygame.MOUSEMOTION:
                x, y = event.pos
                new_pan = x / SCR_W
                if abs(new_pan - pan_angle) > 0.01:
                    pan_angle = new_pan
                    kit.servo[0].angle = 180 - (pan_angle * 180)
                new_tilt = y / SCR_H
                if abs(new_tilt - tilt_angle) > 0.01:
                    tilt_angle = new_tilt
                    kit.servo[1].angle = tilt_angle * 180
        sleep(0.01)

threading.Thread(target=servo_control_thread, daemon=True).start()

# -------------------- MAIN LOOP --------------------
while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)

    cam_pan_deg = pan_angle * 180
    cam_tilt_deg = tilt_angle * 180

    spawn_timer += 1
    if spawn_timer > FRAME_RATE // 3:  
        for _ in range(random.randint(1, 2)): 
            fruits.append(Fruit(cam_pan_deg, fruit_images))
        spawn_timer = 0

    for f in fruits:
        f.update(DT)
        f.draw_on_frame(frame, cam_pan_deg, cam_tilt_deg)
        if f.check_cut(cam_pan_deg, cam_tilt_deg):
            score += 1
    fruits = [f for f in fruits if f.alive]

    cv2.circle(frame, (SCR_W // 2, SCR_H // 2), 8, (0, 0, 255), -1)
    cv2.putText(frame, f"Score: {score}", (10, 30), font, 1, (255, 255, 255), 2)

    cv2.imshow('Camera Fruit Ninja', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
