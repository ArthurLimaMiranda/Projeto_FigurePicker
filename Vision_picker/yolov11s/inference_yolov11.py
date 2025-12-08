import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs
import time
from pathlib import Path

MODEL_PATH = "runs/detect/cubes_spheres_yolo11s/weights/best.pt"
CONFIDENCE_THRESHOLD = 0.5
IMG_SIZE = 640
DEVICE = 'cpu'

print("="*70)
print(" DETECTOR YOLO + REALSENSE D455 (LINUX)")
print("="*70)


print(f"\n Carregando modelo...")
if not Path(MODEL_PATH).exists():
    print(f"❌ Modelo não encontrado: {MODEL_PATH}")
    exit()

model = YOLO(MODEL_PATH)
model.to(DEVICE)

print(f"\n Classes no modelo:")
for idx, name in model.names.items():
    print(f"   ID {idx}: '{name}'")

COLORS = {}
CLASS_NAMES = {}

for idx, name in model.names.items():
    name_lower = name.lower()
    
    if 'cube' in name_lower or 'cubo' in name_lower:
        COLORS[idx] = (255, 0, 0)  # BGR - Azul
        CLASS_NAMES[idx] = 'CUBO'
        print(f"    Mapeado: ID {idx} → CUBO (azul)")
    elif 'sphere' in name_lower or 'esfera' in name_lower or 'ball' in name_lower:
        COLORS[idx] = (0, 255, 0)  # BGR - Verde
        CLASS_NAMES[idx] = 'ESFERA'
        print(f"    Mapeado: ID {idx} → ESFERA (verde)")
    else:
        COLORS[idx] = (255, 255, 255)
        CLASS_NAMES[idx] = name.upper()
        print(f"     ID {idx} → {name}")

print(f"\n✅ Modelo carregado!")

print(f"\n Inicializando RealSense D455...")

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

try:
    profile = pipeline.start(config)
    device = profile.get_device()
    print(f" RealSense: {device.get_info(rs.camera_info.name)}")
    
    print(" Aguardando estabilização...")
    for _ in range(30):
        pipeline.wait_for_frames()
    print("✅ Pronta!")
    
except Exception as e:
    print(f" Erro: {e}")
    exit()

align = rs.align(rs.stream.color)

print("\n" + "="*70)
print(" CONTROLES")
print("="*70)
print("  'q' - Sair")
print("  's' - Salvar screenshot")
print("  '+' - Aumentar confiança")
print("  '-' - Diminuir confiança")
print("  'd' - Toggle depth view")
print("="*70 + "\n")


prev_time = time.time()
frame_count = 0
fps = 0
show_depth = False


def draw_detections(frame, depth_frame, results, conf_threshold):
    """Desenha detecções com informação de debug"""
    detections = []
    
    for result in results:
        for box in result.boxes:
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            
            if conf < conf_threshold:
                continue
            
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            
            distance = depth_frame.get_distance(cx, cy)
            
            model_class_name = model.names[cls]
            display_name = CLASS_NAMES.get(cls, model_class_name)
            
            detections.append({
                'class_id': cls,
                'confidence': conf,
                'bbox': (x1, y1, x2, y2),
                'distance': distance,
                'model_name': model_class_name,
                'display_name': display_name
            })
            
            color = COLORS.get(cls, (255, 255, 255))
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            
            text = f"{display_name} {conf:.1%}"
            if distance > 0:
                text += f" | {distance:.2f}m"
            
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(frame, (x1, y1-th-10), (x1+tw+10, y1), color, -1)
            cv2.putText(frame, text, (x1+5, y1-5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            debug_text = f"[Modelo: '{model_class_name}' ID:{cls}]"
            cv2.putText(frame, debug_text, (x1, y2+25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            if 'sphere' in model_class_name.lower():
                cv2.circle(frame, (cx, cy), 12, color, -1)
                cv2.circle(frame, (cx, cy), 12, (255, 255, 255), 2)
            else:
                s = 12
                cv2.rectangle(frame, (cx-s, cy-s), (cx+s, cy+s), color, -1)
                cv2.rectangle(frame, (cx-s, cy-s), (cx+s, cy+s), (255, 255, 255), 2)
            
            # Cruz de medição
            cv2.drawMarker(frame, (cx, cy), (0, 255, 255), 
                          cv2.MARKER_CROSS, 25, 2)
    
    return detections

def draw_info_panel(frame, detections, fps, conf):
    """Painel de informações"""
    h, w = frame.shape[:2]
    overlay = frame.copy()
    
    cv2.rectangle(overlay, (0, 0), (w, 120), (40, 40, 40), -1)
    frame = cv2.addWeighted(overlay, 0.75, frame, 0.25, 0)
    
    cubos = sum(1 for d in detections if 'CUBO' in d['display_name'])
    esferas = sum(1 for d in detections if 'ESFERA' in d['display_name'])
    
    cv2.putText(frame, f"FPS: {fps:.1f}", (15, 35),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(frame, f"Confianca: {conf:.2f}", (15, 70),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    cv2.putText(frame, f"Total: {len(detections)}", (15, 100),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    
    cv2.putText(frame, f"Cubos: {cubos}", (280, 50),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    cv2.putText(frame, f"Esferas: {esferas}", (280, 90),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    lx = w - 200
    cv2.rectangle(frame, (lx, 20), (lx+30, 50), (255, 0, 0), -1)
    cv2.putText(frame, "= Cubo", (lx+40, 42),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    cv2.rectangle(frame, (lx, 60), (lx+30, 90), (0, 255, 0), -1)
    cv2.putText(frame, "= Esfera", (lx+40, 82),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    return frame


print(" Detector ativo!\n")
print(" Observe o texto amarelo '[Modelo: ...]' embaixo de cada detecção")
print("   Isso mostra o que o modelo REALMENTE detectou\n")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            continue
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        results = model.predict(
            color_image,
            conf=CONFIDENCE_THRESHOLD,
            imgsz=IMG_SIZE,
            verbose=False,
            device=DEVICE
        )
        
        detections = draw_detections(color_image, depth_frame, results, 
                                    CONFIDENCE_THRESHOLD)
        
        if detections and frame_count % 30 == 0:
            print(f"\n Frame {frame_count} - {len(detections)} detecção(ões):")
            for i, d in enumerate(detections, 1):
                print(f"   [{i}] {d['display_name']:8} | "
                      f"Modelo diz: '{d['model_name']:8}' (ID:{d['class_id']}) | "
                      f"Conf: {d['confidence']:.1%} | "
                      f"Dist: {d['distance']:.2f}m")
        
        frame_count += 1
        if frame_count % 10 == 0:
            current = time.time()
            fps = 10 / (current - prev_time)
            prev_time = current
        
        color_image = draw_info_panel(color_image, detections, fps, 
                                      CONFIDENCE_THRESHOLD)
        
        if show_depth:
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            display = np.hstack((color_image, depth_colormap))
            window_name = 'YOLO + RealSense (RGB + Depth) - DEBUG MODE'
        else:
            display = color_image
            window_name = 'YOLO + RealSense - DEBUG MODE'
        
        cv2.imshow(window_name, display)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\n Encerrando...")
            break
        elif key == ord('s'):
            fn = f"debug_detection_{int(time.time())}.jpg"
            cv2.imwrite(fn, display)
            print(f" Screenshot salvo: {fn}")
        elif key == ord('+'):
            CONFIDENCE_THRESHOLD = min(0.99, CONFIDENCE_THRESHOLD + 0.05)
            print(f" Confiança: {CONFIDENCE_THRESHOLD:.2f}")
        elif key == ord('-'):
            CONFIDENCE_THRESHOLD = max(0.01, CONFIDENCE_THRESHOLD - 0.05)
            print(f" Confiança: {CONFIDENCE_THRESHOLD:.2f}")
        elif key == ord('d'):
            show_depth = not show_depth
            status = "Ligado" if show_depth else "Desligado"
            print(f" Depth view: {status}")

except KeyboardInterrupt:
    print("\n  Interrompido pelo usuário")

except Exception as e:
    print(f"\n❌ Erro: {e}")
    import traceback
    traceback.print_exc()

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("\n✅ Pipeline encerrada")
    print(" Resumo da sessão:")
    print(f"   Total de frames: {frame_count}")