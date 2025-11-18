import cv2
import numpy as np
import joblib
from tkinter import Tk, filedialog
from skimage.transform import resize


IMG_SIZE = (64, 64)
LABEL_MAP = {0: "circle", 1: "square"}


def load_image(image_path):
    """Carrega, converte para grayscale, redimensiona e achata."""
    # Lê em grayscale
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    if image is None:
        raise ValueError("Erro ao carregar imagem!")

    # Redimensiona
    image_resized = resize(image, IMG_SIZE, anti_aliasing=True)

    # Flatten → vira vetor 4096
    return image_resized.flatten()


def classify_image(model, image_array):
    prediction = model.predict([image_array])[0]
    return LABEL_MAP[prediction]


def main():

    # Carrega modelo treinado
    #TODO:POR NUM .ENV DPS
    model = joblib.load("models/melhor_modelo.pkl")

    # Tk para abrir arquivo
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

    # Processa imagem
    image_data = load_image(image_path)

    # Classifica
    prediction = classify_image(model, image_data)

    print(f"\n >>> A imagem é classificada como: {prediction}\n")


if __name__ == "__main__":
    main()
