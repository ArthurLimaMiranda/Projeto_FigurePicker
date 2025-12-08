import cv2
import numpy as np
import matplotlib.pyplot as plt
from tkinter import Tk, filedialog
import joblib
from skimage.transform import resize


#=========VARS==========
#TODO:POR NUM .ENV DPS
save_path = '../Classifier/models/melhor_modelo.pkl'

model = joblib.load(save_path)

label_map = {0:'circle',1:'square'}

#=========Defs and Functions=========

def distance(p1,p2):
    '''Calcular a distância entre dois pontos'''
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def classify_shape(roi_gray):
    '''Classificar a Figura'''

    #Converter para Grayscale
    roi_resized = resize(roi_gray, (64, 64),anti_aliasing=True)
    #Normalizar IMG
    roi_flat = roi_resized.flatten().reshape(1,-1)

    # Faz a predição da IMG
    pred = model.predict(roi_flat)[0]
    return label_map[pred]


#=========Selecting Image=========
# Abrindo o arquivo
root = Tk()
root.withdraw()

image_path = filedialog.askopenfilename(
    title='Selecione a Figura',
    filetypes=[("Image files", "*.jpg;*.png;*.jpeg")]
)

#Limpando a memória
root.destroy()

if not image_path:
    print("Nenhum arquivo selecionado")
    exit()

# Carregando a Figura
img  = cv2.imread(image_path)
# Convertendo para Grayscale
gray = cv2.cvtColor(img , cv2.COLOR_BGR2GRAY)
# Aplicando Blur
blur = cv2.GaussianBlur(gray, (5, 5), 0)

#Threshold
_,thresh = cv2.threshold(
        blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

#Acha Bordas/Contornos
contours, _ = cv2.findContours(
    thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
)

#=========Prompt User=========
valid_shapes = list(label_map.values())

target_shape = None

while target_shape is None:
    resposta = input("Digite a figura desejada (circle ou square): ").lower()
    if resposta in valid_shapes:
        target_shape = resposta
    else:
        print("Opção inválida, tente novamente.")


min_dist = 20
detected_points = []


#=========Main Logica=========

for contour in contours:
    x,y,w,h = cv2.boundingRect(contour)

    #ignora contornos muito pequenos
    if w < min_dist or h < min_dist:
        continue

    roi_gray = gray[y:y+h, x:x+w]


    #Classificar a Figura
    shape = classify_shape(roi_gray)

    if shape != target_shape:
        continue
    
    #Centro da Figura
    cx = x + w/2
    cy = y + h/2

    valid = True
    for p in detected_points:
        #Checa se a dist é válida
        if distance(p["position"], (cx,cy)) < min_dist:
            valid = False
            break

        if not valid:
            continue

        #Se for válido adiciona a posição
        detected_points.append({
            "shape": shape,
            "position": (cx,cy)
        })

        cv2.retangle(
            img,(x,y),(x+w,y+h),
            color=(0,0,255),
            thickness=2
        )
        cv2.putText(
            img,shape,(x,y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,(0,0,255),2
        )
     
#==========Resultados=========

print(f'\nFiguras encontradas: {detected_points}\n')

#Mostrando a Figura
cv2.imshow("Detected Figures", img)
cv2.waitKey(0)
cv2.destroyAllWindows()


#=========Plotando=========
fig, ax = plt.subplots()

for p in detected_points:
    ax.scatter(p["position"][0], -p["position"][1])
    ax.text(p["position"][0], -p["position"][1], f"{p['position']}", fontsize=8)

plt.title(f"Posições de '{target_shape}'")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()