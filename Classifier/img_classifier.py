import cv2
import numpy as np
import joblib
from tkinter import Tk, filedialog
from skimage.transform import resize


IMG_SIZE = (64, 64)
LABEL_MAP = {0: "cubo", 1: "esfera"}


def load_image(image_path, use_color=True):
    """Carrega e processa a imagem para classificação"""
    # Verifica cores
    if use_color:
        # Carregar em cores
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError("Erro ao carregar imagem!")
        
        # Converte BGR para RGB 
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    else:
        # Carrega em grayscale
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError("Erro ao carregar imagem!")
    
    # Redimensionar para 64x64 
    image_resized = cv2.resize(image, (64, 64))
    
    # Achata
    return image_resized.flatten()


def classify_image(model, image_array):
    prediction = model.predict([image_array])[0]
    return prediction

def show_image_with_prediction(image_path, prediction, label_map):
    """Mostra a imagem com a predição"""
    # Carregar imagem
    img_display = cv2.imread(image_path)
    #Verifica se a imagem foi carregada
    if img_display is not None:
        # add texto com a predição
        label_text = f"Class: {label_map.get(prediction, f'Classe {prediction}')}"
        cv2.putText(img_display, label_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Redimensiona
        height, width = img_display.shape[:2]
        scale = min(800 / width, 600 / height)
        new_width = int(width * scale)
        new_height = int(height * scale)
        img_display = cv2.resize(img_display, (new_width, new_height))
        
        cv2.imshow("Classificação", img_display)
        print("\nPressione qualquer tecla na janela da imagem para fechar...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():

    # Carrega modelo 
    #TODO:POR NUM .ENV DPS
    model = joblib.load("models/melhor_modelo_open_cv.pkl")

    root = Tk()
    root.withdraw()

    image_path = filedialog.askopenfilename(
        title='Selecione a Figura',
        filetypes=[("Image files", "*.jpg;*.png;*.jpeg")]
    )
    root.destroy()

    if not image_path:
        print("Nenhuma imagem selecionada.")
        return

    use_color = True

    # Processa imagem
    print(f"Processando imagem: {image_path}")
    image_data = load_image(image_path, use_color=use_color)
    
    # Classifica
    prediction = classify_image(model, image_data)
    
    # Mostra resultado
    predicted_label = LABEL_MAP.get(prediction, f"Classe {prediction}")
    print(f"\n{'='*50}")
    print(f">>> A imagem é classificada como: {predicted_label}")
    print(f"{'='*50}\n")
    
    # Mostrar imagem com resultado
    show_image_with_prediction(image_path, prediction, LABEL_MAP)


if __name__ == "__main__":
    main()
