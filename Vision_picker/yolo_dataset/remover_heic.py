# preparar_anotacao_final.py
from pathlib import Path
import shutil
import os

def preparar_para_anotacao():
    """Prepara tudo para anotação no LabelMe"""
    
    print("🎯 PREPARANDO PARA ANOTAÇÃO NO LABELME")
    print("=" * 50)
    
    # 1. Verifica estrutura
    base = Path(".")
    
    if not Path("images/train").exists() or not Path("images/test").exists():
        print("❌ Pastas images/train/ ou images/test/ não encontradas!")
        return
    
    # 2. Conta imagens
    train_images = list(Path("images/train").glob("*.jpg"))
    test_images = list(Path("images/test").glob("*.jpg"))
    
    print(f"\n📊 IMAGENS ENCONTRADAS:")
    print(f"   Train: {len(train_images)} imagens")
    print(f"   Test:  {len(test_images)} imagens")
    
    if len(train_images) == 0 and len(test_images) == 0:
        print("❌ Nenhuma imagem JPG encontrada!")
        return
    
    # 3. Cria pastas para anotação
    Path("labelme_train").mkdir(exist_ok=True)
    Path("labelme_test").mkdir(exist_ok=True)
    
    # 4. Copia imagens para anotação
    print(f"\n📁 COPIANDO IMAGENS PARA ANOTAÇÃO...")
    
    for img in train_images:
        shutil.copy(img, Path("labelme_train") / img.name)
        print(f"  📝 {img.name} → labelme_train/")
    
    for img in test_images:
        shutil.copy(img, Path("labelme_test") / img.name)
        print(f"  📝 {img.name} → labelme_test/")
    
    # 5. Verifica labels.txt
    if not Path("labels.txt").exists():
        print("\n⚠️  labels.txt não encontrado! Criando...")
        with open("labels.txt", "w", encoding="utf-8") as f:
            f.write("cubo\nesfera\n")
        print("✅ labels.txt criado com: cubo, esfera")
    
    # 6. Cria scripts .bat para Windows
    criar_scripts_windows()
    
    # 7. Cria script de conversão
    criar_script_conversao()
    
    print(f"\n" + "=" * 50)
    print("✅ TUDO PRONTO!")
    print("=" * 50)
    
    print(f"\n🎯 SIGA ESTA ORDEM:")
    print(f"\n1️⃣  ANOTAR TREINO:")
    print(f'   Execute: labelme_train\\1_anotar_treino.bat')
    print(f'   OU: labelme labelme_train --output labelme_train --labels labels.txt --autosave')
    
    print(f"\n2️⃣  ANOTAR TESTE:")
    print(f'   Execute: labelme_test\\2_anotar_teste.bat')
    
    print(f"\n3️⃣  CONVERTER PARA YOLO:")
    print(f'   Execute: 3_converter_yolo.bat')
    
    print(f"\n4️⃣  TREINAR MODELO:")
    print(f'   Execute: 4_treinar_yolo.bat')

def criar_scripts_windows():
    """Cria scripts .bat para Windows"""
    
    print(f"\n📜 CRIANDO SCRIPTS .BAT...")
    
    # Script 1: Anotar treino
    with open("labelme_train/1_anotar_treino.bat", "w", encoding="utf-8") as f:
        f.write("""@echo off
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
labelme "labelme_train" --output "labelme_train" --labels "labels.txt" --autosave
echo.
echo Anotacao concluida! Pressione qualquer tecla para sair...
pause > nul
""")
    
    # Script 2: Anotar teste
    with open("labelme_test/2_anotar_teste.bat", "w", encoding="utf-8") as f:
        f.write("""@echo off
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
labelme "labelme_test" --output "labelme_test" --labels "labels.txt" --autosave
echo.
echo Anotacao concluida! Pressione qualquer tecla para sair...
pause > nul
""")
    
    print("   ✅ Scripts de anotação criados")

def criar_script_conversao():
    """Cria scripts para converter e treinar"""
    
    # Script 3: Converter para YOLO
    with open("3_converter_yolo.bat", "w", encoding="utf-8") as f:
        f.write("""@echo off
chcp 65001 > nul
cls
echo.
echo ========================================
echo   CONVERTER PARA FORMATO YOLO
echo ========================================
echo.
echo Convertendo anotacoes LabelMe para YOLO...
echo.
python converter_labelme_to_yolo.py
echo.
echo Pressione qualquer tecla para continuar...
pause > nul
""")
    
    # Script 4: Treinar YOLO
    with open("4_treinar_yolo.bat", "w", encoding="utf-8") as f:
        f.write("""@echo off
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
python treinar_yolov11.py
echo.
echo Treino concluido! Pressione qualquer tecla para sair...
pause > nul
""")
    
    print("   ✅ Scripts de conversão e treino criados")

if __name__ == "__main__":
    preparar_para_anotacao()