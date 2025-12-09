from ultralytics import YOLO
import numpy as np
import pyrealsense2 as rs
import cv2
import random
import torch
from torch.nn import Module, Conv2d, ReLU
from torch.nn import functional as F
from torch.nn import ModuleList, MaxPool2d, ConvTranspose2d
from torchvision.transforms import CenterCrop
import threading
import sys
import serial
import json

torch.set_num_threads(8)

class Block(Module):
    def __init__(self, inChannels, outChannels):
        super().__init__()
        self.conv1 = Conv2d(inChannels, outChannels, 3)
        self.relu = ReLU()
        self.conv2 = Conv2d(outChannels, outChannels, 3)

    def forward(self, x):
        return self.conv2(self.relu(self.conv1(x)))
    
class Encoder(Module):
    def __init__(self, channels):
        super().__init__()
        self.encBlocks = ModuleList([Block(channels[i], channels[i + 1]) for i in range(len(channels) - 1)])
        self.pool = MaxPool2d(2)

    def forward(self, x):
        blockOutputs = []
        for block in self.encBlocks:
            x = block(x)
            blockOutputs.append(x)
            x = self.pool(x)
        return blockOutputs

class Decoder(Module):
    def __init__(self, channels):
        super().__init__()
        self.channels = channels
        self.upconvs = ModuleList(
            [ConvTranspose2d(channels[i], channels[i + 1], 2, 2) for i in range(len(channels) - 1)])
        self.dec_blocks = ModuleList([Block(channels[i], channels[i + 1]) for i in range(len(channels) - 1)])

    def forward(self, x, encFeatures):
        for i in range(len(self.channels) - 1):
            x = self.upconvs[i](x)
            encFeat = self.crop(encFeatures[i], x)
            x = torch.cat([x, encFeat], dim=1)
            x = self.dec_blocks[i](x)
        return x

    def crop(self, encFeatures, x):
        (_, _, H, W) = x.shape
        encFeatures = CenterCrop([H, W])(encFeatures)
        return encFeatures
    
class UNet(Module):
    def __init__(self, encChannels, decChannels, outSize, nbClasses=1, retainDim=True):
        super().__init__()
        self.encoder = Encoder(encChannels)
        self.decoder = Decoder(decChannels)
        self.head = Conv2d(decChannels[-1], nbClasses, 1)
        self.retainDim = retainDim
        self.outSize = outSize

    def forward(self, x):
        encFeatures = self.encoder(x)
        decFeatures = self.decoder(encFeatures[::-1][0], encFeatures[::-1][1:])
        map_ = self.head(decFeatures)
        if self.retainDim:
            map_ = F.interpolate(map_, self.outSize)
        return map_

class Plane:
    def __init__(self):
        self.inliers = []
        self.equation = []

    def fit(self, pts, thresh=0.05, minPoints=100, maxIteration=1000):
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []

        for it in range(maxIteration):
            id_samples = random.sample(range(0, n_points), 3)
            pt_samples = pts[id_samples]

            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecB = pt_samples[2, :] - pt_samples[0, :]
            vecC = np.cross(vecA, vecB)
            
            norm = np.linalg.norm(vecC)
            if norm < 1e-6:
                continue
                
            vecC = vecC / norm
            k = -np.sum(np.multiply(vecC, pt_samples[1, :]))
            if k < 0:
                k *= -1

            plane_eq = [vecC[0], vecC[1], vecC[2], k]

            dist_pt = (
                plane_eq[0] * pts[:, 0] + plane_eq[1] * pts[:, 1] + plane_eq[2] * pts[:, 2] + plane_eq[3]
            ) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
            if len(pt_id_inliers) > len(best_inliers):
                best_eq = plane_eq
                best_inliers = pt_id_inliers
            self.inliers = best_inliers
            self.equation = best_eq

        return self.equation, self.inliers

def calculate_center_xyz(frameDepth, mask, vertices):
    mask_copy = mask.copy()
    mask_copy[frameDepth < 350] = False
    mask_copy[frameDepth > 4500] = False
    mask_copy[frameDepth == 0] = False

    vertices_list = vertices[mask_copy == 255]

    if vertices_list.shape[0] < 50:
        return np.array([-1, -1, -1])

    mean_point = np.mean(vertices_list, axis=0)
    std_point = np.std(vertices_list, axis=0)
    
    distances = np.abs(vertices_list - mean_point)
    mask_outliers = np.all(distances < 2 * std_point, axis=1)
    vertices_filtered = vertices_list[mask_outliers]
    
    if vertices_filtered.shape[0] < 50:
        vertices_filtered = vertices_list

    plane1 = Plane()
    best_eq, best_inliers = plane1.fit(vertices_filtered, thresh=0.02, maxIteration=2000)

    if len(best_inliers) < 20:
        return np.array([-1, -1, -1])

    median_x = np.median(vertices_filtered[best_inliers, 0])
    median_y = np.median(vertices_filtered[best_inliers, 1])
    median_z = np.median(vertices_filtered[best_inliers, 2])

    center = np.array([median_x, median_y, median_z])
    
    for iteration in range(5):
        radius = 0.15 / (iteration + 1)
        
        dist = np.linalg.norm(vertices_filtered[best_inliers] - center, axis=1)
        new_points_mask = dist < radius
        new_points = best_inliers[new_points_mask]

        if len(new_points) > 10:
            weights = 1.0 / (dist[new_points_mask] + 0.001)
            center[0] = np.average(vertices_filtered[new_points, 0], weights=weights)
            center[1] = np.average(vertices_filtered[new_points, 1], weights=weights)
            center[2] = np.average(vertices_filtered[new_points, 2], weights=weights)
        else:
            break

    distances_to_center = np.linalg.norm(vertices_filtered[best_inliers] - center, axis=1)
    if np.min(distances_to_center) > 0.2:
        center[0] = np.median(vertices_filtered[best_inliers, 0])
        center[1] = np.median(vertices_filtered[best_inliers, 1])
        center[2] = np.median(vertices_filtered[best_inliers, 2])
    
    return center
    
def segment_unet(frame, depth, points):
    UNET_IMAGE_WIDTH = 64
    UNET_IMAGE_HEIGHT = 64
    DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
    unet_threshold = 0.3
    
    use_rgb_depth = 3

    if use_rgb_depth == 3:
        in_channels = 4

    encChannels = (in_channels, 1)
    decChannels = (1,)

    unet = UNet(encChannels=encChannels, decChannels=decChannels, outSize=(UNET_IMAGE_HEIGHT, UNET_IMAGE_WIDTH)).to(DEVICE)

    points[points < 0] = 0

    rgb = frame.astype(np.float32)
    rgb = rgb / 255.0
    depth_norm = depth.astype(np.float32)

    top = int(points[3])
    if top >= frame.shape[0]:
        top = frame.shape[0] - 1

    bottom = int(points[1])
    left = int(points[0])
    right = int(points[2])
    if right >= frame.shape[1]:
        right = frame.shape[1] - 1

    cropped_rgb = rgb[bottom:top, left:right].copy()
    cropped_rgb = cv2.resize(cropped_rgb, (UNET_IMAGE_WIDTH, UNET_IMAGE_HEIGHT))

    cropped_depth = depth_norm[bottom:top, left:right].copy()
    cropped_depth = cv2.resize(cropped_depth, (UNET_IMAGE_WIDTH, UNET_IMAGE_HEIGHT))
    cropped_depth = (cropped_depth - np.amin(cropped_depth)) / (
                np.amax(cropped_depth) - np.amin(cropped_depth) + 0.000001)

    cropped_image = np.dstack((cropped_rgb, cropped_depth))

    cv2.imshow("cropped_image", cropped_image)

    image = np.transpose(cropped_image, (2, 0, 1))
    image = np.expand_dims(image, 0)
    image = torch.from_numpy(image).to(DEVICE)

    predMask = unet(image).squeeze()
    predMask = torch.sigmoid(predMask)
    predMask = predMask.detach().cpu().numpy()

    predMask = (predMask > unet_threshold) * 255
    predMask = predMask.astype(np.uint8)

    kernel = np.ones((3, 3), np.uint8)
    predMask = cv2.morphologyEx(predMask, cv2.MORPH_CLOSE, kernel, iterations=2)
    predMask = cv2.morphologyEx(predMask, cv2.MORPH_OPEN, kernel, iterations=1)

    predMask = cv2.resize(predMask, (right - left, top - bottom))

    cv2.imshow("predMask", predMask)

    blank = np.zeros(shape=(frame.shape[0], frame.shape[1]), dtype=np.float32)

    blank[bottom:top, left:right] = predMask

    return blank

def input_thread(detection_data, running, ser):
    while running[0]:
        try:
            sys.stdout.write("\nDigite 0 para ver esferas ou 1 para ver cubos (ou 'q' para sair): ")
            sys.stdout.flush()
            
            user_input = input()
            
            if user_input.lower() == 'q':
                running[0] = False
                break
            
            if user_input == '0':
                if detection_data['sphere']:
                    result = {"esfera": detection_data['sphere']}
                    print(result)
                    
                    # Enviar para ESP32
                    if ser:
                        json_str = json.dumps(result)
                        ser.write((json_str + '\n').encode())
                        print(f"Enviado para ESP32: {json_str}")
                else:
                    print("Nenhuma esfera detectada.")
                
            elif user_input == '1':
                if detection_data['cube']:
                    result = {"cubo": detection_data['cube']}
                    print(result)
                    
                    # Enviar para ESP32
                    if ser:
                        json_str = json.dumps(result)
                        ser.write((json_str + '\n').encode())
                        print(f"Enviado para ESP32: {json_str}")
                else:
                    print("Nenhum cubo detectado.")
                
        except Exception as e:
            break

# Inicializar comunicação serial
try:
    ser = serial.Serial('COM5', 115200, timeout=1)
    print("Conexão serial estabelecida em COM5")
except Exception as e:
    print(f"Erro ao conectar à porta serial: {e}")
    ser = None

detection_data = {
    'sphere': None,
    'cube': None
}

running = [True]

input_thread_obj = threading.Thread(target=input_thread, args=(detection_data, running, ser), daemon=True)
input_thread_obj.start()

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe_profile = pipe.start(cfg)
align_to = rs.stream.color
align = rs.align(align_to)
colorizer = rs.colorizer()
pc = rs.pointcloud()

depth_sensor = pipe_profile.get_device().first_depth_sensor()
preset_range = depth_sensor.get_option_range(rs.option.visual_preset)

weights_path = 'best_new.pt' 
model = YOLO(weights_path)
model.verbose = False

classes = ['sphere', 'cube']

try:
    while running[0]:
        last_sphere = None
        last_cube = None
        
        frames = pipe.wait_for_frames()
        frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        points = pc.calculate(depth_frame)
        depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height
        vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(h, w, 3)

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        colorized_depth = colorizer.colorize(depth_frame)
        depth_cm = np.asanyarray(colorized_depth.get_data())

        results = model.track(color_image, persist=True, verbose=False)

        annotated_frame = results[0].plot()
        cv2.imshow('YOLOv11 RealSense', annotated_frame)

        for i, result in enumerate(results):
            boxes = result.boxes
            
            for box in boxes:
                xyxy = box.xyxy.tolist()[0]
                x1, y1, x2, y2 = xyxy

                class_id = int(box.cls)
                confidence = float(box.conf)
                class_name = model.names[class_id]

                if class_name == 'sphere':
                    mask = segment_unet(color_image, depth_image, np.array(xyxy))
                    cv2.imshow("mask", mask)
                    
                    xyz_center = calculate_center_xyz(depth_image, mask, vertices)
                    
                    if xyz_center[0] != -1:
                        last_sphere = [round(float(xyz_center[0]), 4), 
                                      round(float(xyz_center[1]), 4), 
                                      round(float(xyz_center[2]), 4)]
                
                if class_name == 'cube':
                    mask = segment_unet(color_image, depth_image, np.array(xyxy))
                    cv2.imshow("mask", mask)
                    
                    xyz_center = calculate_center_xyz(depth_image, mask, vertices)
                    
                    if xyz_center[0] != -1:
                        last_cube = [round(float(xyz_center[0]), 4), 
                                    round(float(xyz_center[1]), 4), 
                                    round(float(xyz_center[2]), 4)]
        
        if last_sphere is not None:
            detection_data['sphere'] = last_sphere
        
        if last_cube is not None:
            detection_data['cube'] = last_cube

        if cv2.waitKey(1) & 0xFF == ord('q'):
            running[0] = False
            break
finally:
    pipe.stop()
    cv2.destroyAllWindows()
    if ser:
        ser.close()
    print("\nSistema encerrado.")