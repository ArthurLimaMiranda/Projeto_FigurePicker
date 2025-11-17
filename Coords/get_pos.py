import cv2
import numpy as np
import matplotlib.pyplot as plt
from tkinter import Tk, filedialog  


def is_white(color):
    # Verifica se a cor está próxima do branco
    return np.all(color > 200)


def distance(p1, p2):
    # Calcula a distância euclidiana entre dois pontos
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# Abre a caixa de diálogo de arquivo para escolher o arquivo de imagem
root = Tk()
root.withdraw()
image_path = filedialog.askopenfilename(title="Select Image File", filetypes=[("Image files", "*.jpg;*.jpeg;*.png")])
root.destroy()

if not image_path:
    print("Nenhum arquivo selecionado. Encerrando o programa.")
    exit()

# Le a imagem
image = cv2.imread(image_path)

# Converte para HSV
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define intervalos de cores em HSV
color_ranges = {
    "purple": ((125, 50, 50), (160, 255, 255))
}

# Distância mínima entre pontos
min_distance = 20

# Inicializa uma lista para armazenar posições centrais
colored_points = []

# Itera através de intervalos de cores
for color, (lower_bound, upper_bound) in color_ranges.items():
    # Limita a imagem para obter uma máscara binária
    mask = cv2.inRange(hsv_image, np.array(lower_bound, dtype=np.uint8), np.array(upper_bound, dtype=np.uint8))

    # Acha os contornos
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Itera através de contornos
    for contour in contours:
        # Calcula a caixa delimitadora para o contorno
        x, y, w, h = cv2.boundingRect(contour)

        # Pula a cor branca
        if is_white(image[y:y + h, x:x + w]):
            continue

        # Obtém a posição central e a cor RGB
        center_x = x + w // 2
        center_y = y + h // 2
        rgb_color = tuple(image[center_y, center_x])
        # print(rgb_color)

        # Verifica a distância mínima entre os pontos
        valid_point = True
        for point in colored_points:
            if distance(point['position'], (center_x, center_y)) < min_distance:
                valid_point = False
                break

        # Se o ponto for válido, anexa à lista
        if valid_point:
            colored_points.append({
                'color_rgb': rgb_color,
                'position': (center_x, center_y),
            })

            # Desenha uma caixa delimitadora ao redor do ponto
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

print(colored_points)
# Mostra o resultado
cv2.imshow('Pontos coloridos detectados com caixas delimitadoras', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Plota as coordenadas
fig, ax = plt.subplots()
for point in colored_points:
    # Inverte as componentes de cor de RGB para BGR
    bgr_color = (point['color_rgb'][2], point['color_rgb'][1], point['color_rgb'][0])

    ax.scatter(point['position'][0], -point['position'][1], color=np.array(bgr_color) / 255)
    ax.text(point['position'][0], -point['position'][1], f"{point['position']}", fontsize=8, ha='right',
            color=np.array(bgr_color) / 255)

plt.title('Pontos coloridos detectados com coordenadas')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()