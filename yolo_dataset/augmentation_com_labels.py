# augmentation_corrigido.py
import cv2
import albumentations as A
import numpy as np
from pathlib import Path
import shutil
import yaml

def augment_with_labels_corrigido(images_dir, labels_dir, output_dir, augmentations_per_image=5):
    """
    Augmentation CORRIGIDA - formato correto de bounding boxes
    """
    print("🔄 AUGMENTATION CORRIGIDA")
    print("=" * 50)
    
    images_dir = Path(images_dir)
    labels_dir = Path(labels_dir)
    output_dir = Path(output_dir)
    
    # Cria diretórios
    (output_dir / "images").mkdir(parents=True, exist_ok=True)
    (output_dir / "labels").mkdir(parents=True, exist_ok=True)
    
    # Carrega classes
    with open("dataset.yaml", 'r') as f:
        dataset_config = yaml.safe_load(f)
    classes = dataset_config['names']
    print(f"📋 Classes: {classes}")
    
    # Transformações CORRIGIDAS
    transform = A.Compose([
        A.HorizontalFlip(p=0.5),
        A.VerticalFlip(p=0.2),
        A.Rotate(limit=15, p=0.5, border_mode=cv2.BORDER_CONSTANT, value=0),
        A.RandomScale(scale_limit=0.2, p=0.3),
        
        A.RandomBrightnessContrast(brightness_limit=0.2, contrast_limit=0.2, p=0.5),
        A.HueSaturationValue(hue_shift_limit=20, sat_shift_limit=30, val_shift_limit=20, p=0.5),
        A.RGBShift(r_shift_limit=20, g_shift_limit=20, b_shift_limit=20, p=0.3),
        
        A.GaussNoise(var_limit=(10.0, 50.0), p=0.3),
        A.GaussianBlur(blur_limit=(3, 7), p=0.3),
        A.MotionBlur(blur_limit=7, p=0.2),
        
        A.CoarseDropout(max_holes=8, max_height=32, max_width=32, p=0.3),
    ], bbox_params=A.BboxParams(
        format='albumentations',  # ← CORRIGIDO! Formato [x_min, y_min, x_max, y_max]
        label_fields=['class_labels'],
        min_area=16,
        min_visibility=0.3
    ))
    
    # Processa cada imagem
    image_files = list(images_dir.glob("*.jpg"))
    
    total_original = len(image_files)
    total_augmented = 0
    total_erros = 0
    
    print(f"\n📁 Processando {total_original} imagens...")
    
    for img_idx, img_file in enumerate(image_files):
        print(f"\n🖼️  Imagem {img_idx+1}/{total_original}: {img_file.name}")
        
        # Carrega imagem
        image = cv2.imread(str(img_file))
        if image is None:
            print(f"   ❌ Não foi possível ler a imagem")
            continue
            
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width = image.shape[:2]
        
        # Carrega labels YOLO
        label_file = labels_dir / f"{img_file.stem}.txt"
        
        if not label_file.exists():
            print(f"   ⚠️  Label não encontrado: {label_file.name}")
            continue
        
        # Lê bounding boxes e converte para formato Albumentations
        bboxes = []
        class_labels = []
        
        with open(label_file, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 5:
                    class_id = float(parts[0])
                    x_center = float(parts[1])  # Normalizado (0-1)
                    y_center = float(parts[2])  # Normalizado (0-1)
                    bbox_width = float(parts[3])  # Normalizado (0-1)
                    bbox_height = float(parts[4])  # Normalizado (0-1)
                    
                    # Converte YOLO [x_center, y_center, width, height] 
                    # para Albumentations [x_min, y_min, x_max, y_max] (NORMALIZADO!)
                    x_min = x_center - (bbox_width / 2)
                    y_min = y_center - (bbox_height / 2)
                    x_max = x_center + (bbox_width / 2)
                    y_max = y_center + (bbox_height / 2)
                    
                    # Garante que está dentro dos limites 0-1
                    x_min = max(0.0, min(1.0, x_min))
                    y_min = max(0.0, min(1.0, y_min))
                    x_max = max(0.0, min(1.0, x_max))
                    y_max = max(0.0, min(1.0, y_max))
                    
                    # Verifica se o bbox é válido
                    if x_max > x_min and y_max > y_min:
                        bboxes.append([x_min, y_min, x_max, y_max])
                        class_labels.append(class_id)
                    else:
                        print(f"   ⚠️  Bbox inválido ignorado: {x_min},{y_min},{x_max},{y_max}")
        
        if not bboxes:
            print(f"   ⚠️  Nenhum bbox válido")
            continue
        
        print(f"   📦 {len(bboxes)} objetos: {[classes[int(cls)] for cls in class_labels]}")
        
        # Salva a imagem ORIGINAL
        orig_img_name = f"{img_file.stem}_orig.jpg"
        orig_label_name = f"{img_file.stem}_orig.txt"
        
        cv2.imwrite(str(output_dir / "images" / orig_img_name), 
                   cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        shutil.copy(label_file, output_dir / "labels" / orig_label_name)
        
        # Gera imagens aumentadas
        aug_geradas = 0
        
        for aug_idx in range(augmentations_per_image):
            try:
                # Aplica transformação
                transformed = transform(
                    image=image,
                    bboxes=bboxes,
                    class_labels=class_labels
                )
                
                aug_image = transformed['image']
                aug_bboxes = transformed['bboxes']  # Formato [x_min, y_min, x_max, y_max] normalizado
                aug_classes = transformed['class_labels']
                
                # Verifica se ainda temos bboxes após transformação
                if not aug_bboxes:
                    print(f"      ⚠️  Todos bboxes perdidos na transformação {aug_idx+1}")
                    continue
                
                # Nome dos arquivos
                aug_img_name = f"{img_file.stem}_aug{aug_idx}.jpg"
                aug_label_name = f"{img_file.stem}_aug{aug_idx}.txt"
                
                # Salva imagem aumentada
                cv2.imwrite(str(output_dir / "images" / aug_img_name),
                          cv2.cvtColor(aug_image, cv2.COLOR_RGB2BGR))
                
                # Converte de volta para formato YOLO e salva labels
                with open(output_dir / "labels" / aug_label_name, 'w') as f:
                    for bbox, cls in zip(aug_bboxes, aug_classes):
                        x_min, y_min, x_max, y_max = bbox
                        
                        # Converte para YOLO: [x_center, y_center, width, height]
                        x_center = (x_min + x_max) / 2
                        y_center = (y_min + y_max) / 2
                        bbox_width = x_max - x_min
                        bbox_height = y_max - y_min
                        
                        # Valida bounding box
                        if (0 <= x_center <= 1 and 0 <= y_center <= 1 and
                            0.01 < bbox_width <= 1 and 0.01 < bbox_height <= 1):
                            f.write(f"{int(cls)} {x_center:.6f} {y_center:.6f} {bbox_width:.6f} {bbox_height:.6f}\n")
                        else:
                            print(f"      ⚠️  Bbox inválido pós-transform: {bbox}")
                
                aug_geradas += 1
                total_augmented += 1
                print(f"   ✅ Aug {aug_idx+1} gerada ({len(aug_bboxes)} objetos)")
                
            except Exception as e:
                total_erros += 1
                print(f"   ❌ Erro na augmentação {aug_idx+1}: {str(e)[:100]}...")
        
        print(f"   📊 {aug_geradas}/{augmentations_per_image} augmentações bem-sucedidas")
    
    # Estatísticas
    print(f"\n{'='*50}")
    print("📊 ESTATÍSTICAS DA AUGMENTATION:")
    print(f"{'='*50}")
    print(f"   Imagens originais: {total_original}")
    print(f"   Augmentações por imagem: {augmentations_per_image}")
    print(f"   Total de imagens aumentadas: {total_augmented}")
    print(f"   Total final: {total_original + total_augmented}")
    
    if total_original > 0:
        print(f"   Aumento: {(total_augmented/total_original)*100:.1f}%")
    
    if total_erros > 0:
        print(f"   Erros: {total_erros}")
    
    print(f"\n📁 Saída salva em: {output_dir}")
    
    return output_dir

def main():
    """Teste rápido da correção"""
    
    print("🎯 TESTE DA CORREÇÃO DE AUGMENTATION")
    print("=" * 50)
    
    print("\n📋 TESTE COM:")
    print("   1. Dataset de TREINO (para treino real)")
    print("   2. Dataset de TESTE (apenas para teste)")
    
    opcao = input("\nEscolha (1-2): ").strip()
    
    if opcao == "1":
        images_dir = "images/train"
        labels_dir = "labels/train"
        output_dir = "dataset_aumentado_corrigido/train"
        test_mode = False
    else:
        images_dir = "images/test"
        labels_dir = "labels/test"
        output_dir = "dataset_aumentado_corrigido/test"
        test_mode = True
    
    # Teste com apenas 2 augmentações primeiro
    augmentations_per_image = 10 if test_mode else 20
    
    output = augment_with_labels_corrigido(
        images_dir=images_dir,
        labels_dir=labels_dir,
        output_dir=output_dir,
        augmentations_per_image=augmentations_per_image
    )
    
    print(f"\n✅ Teste {'completo' if not test_mode else 'realizado'}!")
    
    if not test_mode:
        print(f"\n🎯 Para treinar com dataset corrigido:")
        print(f'   python treinar_yolov11.py --data "{output}/../dataset_augmented.yaml"')

if __name__ == "__main__":
    main()