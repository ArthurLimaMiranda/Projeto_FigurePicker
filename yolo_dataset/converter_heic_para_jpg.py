# converter_heic_para_jpg.py
from pathlib import Path
import shutil
import random
from PIL import Image
import pillow_heif

def converter_heic_para_jpg(source_dir, output_dir="converted_images"):
    """
    Converte todas imagens .HEIC/.heic para .jpg
    """
    source = Path(source_dir)
    output = Path(output_dir)
    output.mkdir(exist_ok=True)
    
    # Encontra arquivos HEIC
    heic_files = list(source.glob("*.heic")) + list(source.glob("*.HEIC"))
    
    if not heic_files:
        print("❌ Nenhum arquivo .HEIC encontrado!")
        print("   Verifique se os arquivos têm extensão .heic ou .HEIC")
        # Mostra o que tem na pasta
        print(f"\n📁 Conteúdo de {source}:")
        for item in source.iterdir():
            print(f"   - {item.name}")
        return []
    
    print(f"🔍 Encontrados {len(heic_files)} arquivos HEIC")
    
    # Registra o plugin HEIF
    pillow_heif.register_heif_opener()
    
    imagens_convertidas = []
    
    for heic_file in heic_files:
        try:
            # Abre a imagem HEIC
            print(f"  Convertendo: {heic_file.name}")
            
            with Image.open(heic_file) as img:
                # Converte para RGB (HEIC pode ter canal alpha)
                if img.mode in ('RGBA', 'LA', 'P'):
                    img = img.convert('RGB')
                
                # Novo nome (mesmo nome, mas .jpg)
                novo_nome = heic_file.stem + ".jpg"
                novo_caminho = output / novo_nome
                
                # Salva como JPG
                img.save(novo_caminho, "JPEG", quality=95)
                imagens_convertidas.append(novo_caminho)
                
                print(f"    ✅ {heic_file.name} → {novo_nome}")
                
        except Exception as e:
            print(f"    ❌ Erro ao converter {heic_file.name}: {e}")
    
    print(f"\n✅ Conversão completa!")
    print(f"   Convertidas: {len(imagens_convertidas)} imagens")
    print(f"   Salvas em: {output}")
    
    return imagens_convertidas

def distribuir_imagens_convertidas(imagens, train_ratio=0.8):
    """
    Distribui imagens convertidas entre train/test
    """
    if not imagens:
        print("❌ Nenhuma imagem para distribuir!")
        return
    
    # Embaralha
    random.shuffle(imagens)
    
    # Divide
    split_idx = int(len(imagens) * train_ratio)
    train_images = imagens[:split_idx]
    test_images = imagens[split_idx:]
    
    # Pastas de destino
    train_dest = Path("images/train")
    test_dest = Path("images/test")
    
    train_dest.mkdir(parents=True, exist_ok=True)
    test_dest.mkdir(parents=True, exist_ok=True)
    
    print(f"\n📁 Distribuindo imagens convertidas...")
    
    # Copia para train
    for img in train_images:
        shutil.copy(img, train_dest / img.name)
        print(f"  🟢 {img.name} → images/train/")
    
    # Copia para test
    for img in test_images:
        shutil.copy(img, test_dest / img.name)
        print(f"  🔵 {img.name} → images/test/")
    
    print(f"\n✅ Distribuição completa!")
    print(f"   Train: {len(train_images)} imagens")
    print(f"   Test:  {len(test_images)} imagens")
    
    return len(train_images), len(test_images)

def main():
    print("🔄 CONVERSOR HEIC → JPG + DISTRIBUIÇÃO")
    print("=" * 50)
    
    # Pergunta onde estão os HEICs
    print("\n📍 ONDE ESTÃO SUAS IMAGENS .HEIC?")
    print("Exemplo: C:/Users/aynaa/Desktop/imagens/")
    print("Exemplo: C:/Users/aynaa/Downloads/")
    print("Exemplo: ./raw_heic/  (pasta atual)")
    
    caminho_heic = input("\nDigite o caminho da pasta com imagens .HEIC: ").strip()
    
    if not caminho_heic:
        print("\n⚠️  Execute novamente e digite um caminho válido!")
        return
    
    # 1. Converte HEIC para JPG
    print(f"\n🔄 Passo 1: Convertendo HEIC para JPG...")
    imagens_convertidas = converter_heic_para_jpg(caminho_heic)
    
    if not imagens_convertidas:
        return
    
    # 2. Distribui entre train/test
    print(f"\n🔄 Passo 2: Distribuindo entre train/test...")
    distribuir_imagens_convertidas(imagens_convertidas)
    
    print("\n" + "=" * 50)
    print("🎯 PRÓXIMO PASSO:")
    print("Execute o script de preparação para anotação:")
    print("python preparar_anotacao.py")

if __name__ == "__main__":
    main()