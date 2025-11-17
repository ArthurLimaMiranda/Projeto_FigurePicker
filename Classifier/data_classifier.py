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
import json

#=========VARS==========
SEED = 50

input_dir ='Data'

categories = ['circle','square']

#=========DATA PROCESS=========

def process_data(input_dir,categories):
    '''Adicionando Imgs com Resize'''
    data = []
    labels = []

    if not os.path.exists(input_dir):
       print('O diretório {} não existe!'.format(input_dir))
       exit()

    #Adicionando Imgs com Resize
    for category_idx,category in enumerate(categories):
      for file in os.listdir(os.path.join(input_dir,category)):
        img_path = os.path.join(input_dir,category,file)
        img = imread(img_path)
        img = resize(img, (64, 64))
        data.append(img.flatten())
        labels.append(category_idx)
    return np.asarray(data),np.asarray(labels)


#=========CLASSIFIER=========

# Definindo o modelo
classifier = SVC()

parameters =[{
    'gamma':[0.01,0.001,0.0001],
    'C':[1,10,100,1000]
}]


#=========METRICS=========

def show_metrics(y_pred,y_test):
  '''Calcular a métrica de acurácia'''

  print('{}% TP'.format(str(score*100)))

  # Calcular a matriz de confusão
  cm = confusion_matrix(y_test, y_pred)

  plt.figure(figsize=(8, 6))
  sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
              xticklabels=['Pred circles', 'Pred squares'],
              yticklabels=['True circles', 'True squares'])
  plt.title('Matriz de Confusão')
  plt.xlabel('Predito')
  plt.ylabel('Real')
  plt.show()


#=========SAVING=========

def save_model(best_estimator,path_save):
  '''Salva o modelo e os parâmetros'''

  total_path = os.path.join(path_save,'melhor_modelo.pkl')

  # Salvar o modelo
  joblib.dump(best_estimator, total_path)

  # Salvar também os parâmetros e métricas
  resultados = {
      'best_params': grid_search.best_params_,
      'best_score': grid_search.best_score_,
      'best_estimator': 'melhor_modelo.pkl'
  }

  with open('resultados_gridsearch.json', 'w') as f:
      json.dump(resultados, f, indent=4)

if __name__ == '__main__':

  data,labels = process_data(input_dir,categories)

  #Separando Train e Test
  x_train,x_test,y_train,y_test = train_test_split(
    data,
    labels,
    test_size = 0.2,
    stratify = labels,
    random_state = SEED,
    shuffle =True
)

  grid_search = GridSearchCV(classifier,parameters)

  #treinando o modelo
  grid_search.fit(x_train,y_train)

  #Testando o modelo
  best_estimator = grid_search.best_estimator_

  y_pred = best_estimator.predict(x_test)

  score = accuracy_score(y_test,y_pred)

  #Printando as métricas
  show_metrics(y_pred,y_test)
  #Salvando o modelo
  save_model(best_estimator,'Data/')




