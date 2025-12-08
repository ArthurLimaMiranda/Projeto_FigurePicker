# preparar_anotacao_final.py
import os
from pathlib import Path
import shutil

def preparar_para_anotacao():
    """Prepara tudo para anotação no LabelMe"""
    
    print("🎯 PREPARANDO PARA ANOTAÇÃO NO LABELME")
    print("=" * 50)
    
    # 1. Verifica estrutura usando os.path.join
    base_dir = "."
    
    train_dir = os.path.join("images", "train")
    test_dir = os.path.join("images", "test")
    
    if not os.path.exists(train_dir) or not os.path.exists(test_dir):
        print("❌ Pastas images/train/ ou images/test/ não encontradas!")
        return
    
    # 2. Conta imagens
    train_images = list(Path(train_dir).glob("*.jpg"))
    test_images = list(Path(test_dir).glob("*.jpg"))
    
    print(f"\n📊 IMAGENS ENCONTRADAS:")
    print(f"   Train: {len(train_images)} imagens")
    print(f"   Test:  {len(test_images)} imagens")
    
    if len(train_images) == 0 and len(test_images) == 0:
        print("❌ Nenhuma imagem JPG encontrada!")
        return
    
    # 3. Cria pastas para anotação usando os.path.join
    labelme_train = os.path.join("labelme_train")
    labelme_test = os.path.join("labelme_test")
    
    os.makedirs(labelme_train, exist_ok=True)
    os.makedirs(labelme_test, exist_ok=True)
    
    # 4. Copia imagens para anotação
    print(f"\n📁 COPIANDO IMAGENS PARA ANOTAÇÃO...")
    
    for img in train_images:
        dest_path = os.path.join(labelme_train, img.name)
        shutil.copy(img, dest_path)
        print(f"  📝 {img.name} → labelme_train/")
    
    for img in test_images:
        dest_path = os.path.join(labelme_test, img.name)
        shutil.copy(img, dest_path)
        print(f"  📝 {img.name} → labelme_test/")
    
    # 5. Verifica labels.txt usando os.path.join
    labels_file = os.path.join(base_dir, "labels.txt")
    
    if not os.path.exists(labels_file):
        print("\n⚠️  labels.txt não encontrado! Criando...")
        with open(labels_file, "w", encoding="utf-8") as f:
            f.write("cubo\nesfera\n")
        print("✅ labels.txt criado com: cubo, esfera")
    else:
        # Mostra conteúdo atual
        with open(labels_file, "r", encoding="utf-8") as f:
            content = f.read().strip()
        print(f"\n📄 labels.txt encontrado: {content}")
    
    # 6. Cria scripts .bat para Windows
    criar_scripts_windows(base_dir, labelme_train, labelme_test, labels_file)
    
    # 7. Cria script de conversão
    criar_script_conversao(base_dir)
    
    print(f"\n" + "=" * 50)
    print("✅ TUDO PRONTO!")
    print("=" * 50)
    
    print(f"\n🎯 SIGA ESTA ORDEM:")
    print(f"\n1️⃣  ANOTAR TREINO:")
    print(f'   Execute: {os.path.join(labelme_train, "1_anotar_treino.bat")}')
    print(f'   OU: labelme "{labelme_train}" --output "{labelme_train}" --labels "{labels_file}" --autosave')
    
    print(f"\n2️⃣  ANOTAR TESTE:")
    print(f'   Execute: {os.path.join(labelme_test, "2_anotar_teste.bat")}')
    
    print(f"\n3️⃣  CONVERTER PARA YOLO:")
    print(f'   Execute: {os.path.join(base_dir, "3_converter_yolo.bat")}')
    
    print(f"\n4️⃣  TREINAR MODELO:")
    print(f'   Execute: {os.path.join(base_dir, "4_treinar_yolo.bat")}')

def criar_scripts_windows(base_dir, labelme_train, labelme_test, labels_file):
    """Cria scripts .bat para Windows usando os.path.join"""
    
    print(f"\n📜 CRIANDO SCRIPTS .BAT...")
    
    # Script 1: Anotar treino
    bat_train = os.path.join(labelme_train, "1_anotar_treino.bat")
    with open(bat_train, "w", encoding="utf-8") as f:
        f.write(f"""@echo off
chcp 65001 > nul
cls
echo.
echo ========================================
echo   ANOTAR IMAGENS DE TREINO - CUBOS/ESFERAS
echo ========================================
echo.
echo ATALHOS IMPORTANTES:
echo   [Ctrl+R] = Retangulo (para cubos)
echo   [Ctrl+L] = Poligono (para esferas)
echo   [Enter]  = Finalizar forma
echo   [D]      = Duplicar anotacao anterior
echo   [Ctrl+S] = Salvar automaticamente (ativo)
echo   [Ctrl+H] = Mostrar/ocultar anotações
echo.
echo Classes disponiveis:
echo   - cubo    (use retangulo)
echo   - esfera  (use poligono)
echo.
echo Pressione qualquer tecla para abrir o LabelMe...
pause > nul
echo Abrindo LabelMe...
labelme "{labelme_train}" --output "{labelme_train}" --labels "{labels_file}" --autosave
echo.
echo Anotacao concluida! Pressione qualquer tecla para sair...
pause > nul
""")
    
    # Script 2: Anotar teste
    bat_test = os.path.join(labelme_test, "2_anotar_teste.bat")
    with open(bat_test, "w", encoding="utf-8") as f:
        f.write(f"""@echo off
chcp 65001 > nul
cls
echo.
echo ========================================
echo   ANOTAR IMAGENS DE TESTE - CUBOS/ESFERAS
echo ========================================
echo.
echo Pressione qualquer tecla para abrir o LabelMe...
pause > nul
echo Abrindo LabelMe...
labelme "{labelme_test}" --output "{labelme_test}" --labels "{labels_file}" --autosave
echo.
echo Anotacao concluida! Pressione qualquer tecla para sair...
pause > nul
""")
    
    print(f"   ✅ Scripts de anotação criados em:")
    print(f"      {bat_train}")
    print(f"      {bat_test}")

def criar_script_conversao(base_dir):
    """Cria scripts para converter e treinar usando os.path.join"""
    
    # Script 3: Converter para YOLO
    bat_converter = os.path.join(base_dir, "3_converter_yolo.bat")
    with open(bat_converter, "w", encoding="utf-8") as f:
        f.write(f"""@echo off
chcp 65001 > nul
cls
echo.
echo ========================================
echo   CONVERTER PARA FORMATO YOLO
echo ========================================
echo.
echo Convertendo anotacoes LabelMe para YOLO...
echo.
python "{os.path.join(base_dir, 'converter_labelme_to_yolo.py')}"
echo.
echo Pressione qualquer tecla para continuar...
pause > nul
""")
    
    # Script 4: Treinar YOLO
    bat_treinar = os.path.join(base_dir, "4_treinar_yolo.bat")
    with open(bat_treinar, "w", encoding="utf-8") as f:
        f.write(f"""@echo off
chcp 65001 > nul
cls
echo.
echo ========================================
echo   TREINAR MODELO YOLOv11
echo ========================================
echo.
echo Iniciando treino do modelo...
echo.
echo Nota: O treino pode demorar varios minutos ou horas
echo dependendo do numero de imagens e hardware.
echo.
python "{os.path.join(base_dir, 'treinar_yolov11.py')}"
echo.
echo Treino concluido! Pressione qualquer tecla para sair...
pause > nul
""")
    
    print(f"   ✅ Scripts de conversão e treino criados em:")
    print(f"      {bat_converter}")
    print(f"      {bat_treinar}")

if __name__ == "__main__":
    preparar_para_anotacao()