import os
from skimage.io import	imread
from skimage.transform import resize
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.model_selection import GridSearchCV
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
import joblib
import cv2
import json

#=========VARS==========
SEED = 50

# input_dir ='Data'
#data
input_dir_train = '../yolov11s/dataset_balanced/train/images'
input_dir_test= '../yolov11s/dataset_balanced/test/images'


category_keywords = {
    'cubo': ['cube'],  
    'esfera': ['blue--', 'sphere'] 
}

#=========DATA PROCESS=========

def process_data_with_opencv(input_dir, category_keywords):
    '''Adicionando Imgs com Resize usando opencv'''
    
    data = []
    labels = []
    
    if not os.path.exists(input_dir):
        print('O diretório {} não existe!'.format(input_dir))
        exit()
    
    # Mapear índices para categorias
    categories = list(category_keywords.keys())
    print(f"Categorias definidas: {categories}")
    
    # Listar todos os arquivos de imagem
    image_files = [f for f in os.listdir(input_dir) 
                   if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]
    
    print(f"Encontrados {len(image_files)} arquivos de imagem")
    
    for img_file in image_files:
        img_path = os.path.join(input_dir, img_file)
        
        # Determinar a categoria baseada no nome do arquivo
        category_found = None
        category_idx = None
        
        for idx, (category_name, keywords) in enumerate(category_keywords.items()):
            for keyword in keywords:
                if keyword in img_file:
                    category_found = category_name
                    category_idx = idx
                    break
            if category_found:
                break
        
        # Se não encontrou categoria, pular esta imagem
        if category_found is None:
            print(f"Aviso: Não foi possível identificar categoria para {img_file}")
            continue
        
        try:
            # Ler imagem com OpenCV
            img = cv2.imread(img_path)
            if img is None:
                print(f"Erro: Não foi possível ler a imagem {img_file}")
                continue
            
            # Converter BGR para RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # Redimensionar
            img = cv2.resize(img, (64, 64))
            
            # Achatar e adicionar aos dados
            data.append(img.flatten())
            labels.append(category_idx)
            
            print(f"Processado: {img_file[:30]}... -> Categoria: {category_found}")
            
        except Exception as e:
            print(f"Erro ao processar {img_file}: {e}")
    
    print(f"\nTotal processado: {len(data)} imagens")
    print(f"Distribuição: {np.bincount(labels)}")
    
    return np.asarray(data), np.asarray(labels), categories


#=========METRICS=========

def show_metrics(y_pred, y_test, score, categories, display_time=10):
    '''Calcular a métrica de acurácia'''
    print(f"Acurácia: {score*100:.2f}%")

    # Calcular a matriz de confusão
    cm = confusion_matrix(y_test, y_pred)
    
    # Criar labels para a matriz de confusão
    pred_labels = [f'Pred {cat}s' for cat in categories]
    true_labels = [f'True {cat}s' for cat in categories]

    plt.figure(figsize=(8, 6))
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
                xticklabels=pred_labels,
                yticklabels=true_labels)
    plt.title('Matriz de Confusão')
    plt.xlabel('Predito')
    plt.ylabel('Real')
    plt.show(block=False)
    plt.pause(display_time) 
    plt.close()

#=========SAVING=========

def save_model(best_estimator, grid_search, path_save, categories):

    os.makedirs(path_save, exist_ok=True)

    total_path = os.path.join(path_save, 'melhor_modelo_open_cv.pkl')

    joblib.dump(best_estimator, total_path)

    print(f'Salvando modelo...')
    params_path = os.path.join(path_save, 'model_params.json')
    resultados = {
        'best_params': grid_search.best_params_,
        'best_score': float(grid_search.best_score_),
        'categories': categories,
        'category_keywords': category_keywords
    }

    with open(params_path, 'w') as f:
        json.dump(resultados, f, indent=4)
    
    print(f'Parâmetros salvos em: {params_path}')

if __name__ == '__main__':

  data_train, labels_train, categories = process_data_with_opencv(input_dir_train, category_keywords)
  data_test, labels_test, _ = process_data_with_opencv(input_dir_test, category_keywords)

  print(f"\nShape dos dados de treino: {data_train.shape}")
  print(f"Shape dos dados de teste: {data_test.shape}")

  # Separa em conjuntos de treino e teste
  x_train, y_train = data_train, labels_train
  x_test, y_test = data_test, labels_test

  #=========CLASSIFIER=========

  # Definindo o modelo
  classifier = SVC()

  parameters = [{
        'gamma': [0.01, 0.001, 0.0001],
        'C': [1, 10, 100, 1000],
        'kernel': ['rbf', 'linear']
  }]

  grid_search = GridSearchCV(classifier,parameters)

  #treinando o modelo
  print(f"Treinando o modelo...")
  grid_search.fit(x_train,y_train)

  #Testando o modelo
  best_estimator = grid_search.best_estimator_

  y_pred = best_estimator.predict(x_test)
  print(f"calculando  métricas...")
  score = accuracy_score(y_test,y_pred)

  #Printando as métricas
  show_metrics(y_pred, y_test, score,categories,3) 

  #Salvando o modelo
  #TODO:POR NUM .ENV DPS
  PATH_SAVE = 'models/'
  save_model(best_estimator, grid_search, PATH_SAVE, categories)






