-- Script usada na versão da entrega 11-11 do Projeto FigurePicker
function sysCall_init()
    -- Pega as juntas principais
    juntaX = sim.getObject('/JX')
    juntaY = sim.getObject('/JY')
    juntaZ = sim.getObject('/JZ')
    juntaEsq = sim.getObject('/JL')
    juntaDir = sim.getObject('/JR')

    if juntaX == -1 or juntaY == -1 or juntaZ == -1 or juntaEsq == -1 or juntaDir == -1 then
        error("ERRO: Nao consegui encontrar as juntas '/JX', '/JY', '/JZ', '/JL' ou '/JR'")
    end

    -- Ativa motores e controle de posicao
    local juntas = {juntaX, juntaY, juntaZ, juntaEsq, juntaDir}
    for i = 1, #juntas do
        sim.setObjectInt32Param(juntas[i], sim.jointintparam_motor_enabled, 1)
        sim.setObjectInt32Param(juntas[i], sim.jointintparam_ctrl_enabled, 1)
    end

    -- Obtem o sensor de visao
    camera = sim.getObject('/VS')
    if camera == -1 then
        error("ERRO: Nao encontrei o Vision_sensor. Adicione um na cena e nomeie como '/VS'.")
    end

    -- Posicoes iniciais e limites
    alvoX = 0.0
    alvoY = 0.0
    alvoZ = 0.0
    passo = 0.025
    limiteX = {min = -0.175, max = 0.25}
    limiteY = {min = -0.20, max = 0.20}
    limiteZ = {min = -0.24, max = 0.05}

    -- Garra
    aberto = 0.04
    fechado = 0.0
    estadoAberto = true

    -- Cria UI
    xml = [[
        <ui title="Controle da Garra" closeable="true" resizable="false" activate="false" size="320,550">
            <image id="10" position="30,20" size="256,256"/>

            <label text="Alvo X:" position="20,290" />
            <edit id="1" value="0.0" position="80,285" size="60,20"/>
            
            <label text="Alvo Y:" position="160,290" />
            <edit id="2" value="0.0" position="220,285" size="60,20"/>

            <label text="Alvo Z:" position="20,315" />
            <edit id="3" value="0.0" position="80,310" size="60,20"/>
            
            <button text="Mover" on-click="onMoveClicked" position="100,340" size="100,25"/>

            <!-- Setas -->
            <label text="Controle manual XY:" position="90,370" />
            <button text="CIMA" on-click="onUpClicked" position="130,390" size="40,25"/>
            <button text="BAIXO" on-click="onDownClicked" position="130,420" size="40,25"/>
            <button text="ESQUERDA" on-click="onLeftClicked" position="90,420" size="40,25"/>
            <button text="DIREITA" on-click="onRightClicked" position="170,420" size="40,25"/>

            <label text="Controle Z:" position="90,450" />
            <button text="SUBIR" on-click="onZUpClicked" position="90,470" size="60,25"/>
            <button text="DESCER" on-click="onZDownClicked" position="150,470" size="60,25"/>

            <label text="Garra:" position="110,500" />
            <button text="Abrir" on-click="onAbrir" position="70,520" size="80,30"/>
            <button text="Fechar" on-click="onFechar" position="160,520" size="80,30"/>
        </ui>
    ]]

    uiHandle = simUI.create(xml)
    sim.addLog(sim.verbosity_scriptinfos, 'Interface criada com sucesso.')
end


-- === Funcoes de controle manual ===
function onMoveClicked(ui, id)
    local x = tonumber(simUI.getEditValue(uiHandle, 1))
    local y = tonumber(simUI.getEditValue(uiHandle, 2))
    local z = tonumber(simUI.getEditValue(uiHandle, 3))

    if x and y and z then
        alvoX = limitar(x, limiteX.min, limiteX.max)
        alvoY = limitar(y, limiteY.min, limiteY.max)
        alvoZ = limitar(z, limiteZ.min, limiteZ.max)
        atualizarCampos()
        moverGarra()
    else
        sim.addStatusbarMessage("Erro: valores invalidos!")
    end
end

function onLeftClicked(ui, id)
    alvoX = limitar(alvoX - passo, limiteX.min, limiteX.max)
    atualizarCampos()
    moverGarra()
end

function onRightClicked(ui, id)
    alvoX = limitar(alvoX + passo, limiteX.min, limiteX.max)
    atualizarCampos()
    moverGarra()
end

function onUpClicked(ui, id)
    alvoY = limitar(alvoY - passo, limiteY.min, limiteY.max)
    atualizarCampos()
    moverGarra()
end

function onDownClicked(ui, id)
    alvoY = limitar(alvoY + passo, limiteY.min, limiteY.max)
    atualizarCampos()
    moverGarra()
end

function onZUpClicked(ui, id)
    alvoZ = limitar(alvoZ + 2*passo, limiteZ.min, limiteZ.max)
    atualizarCampos()
    moverGarra()
end

function onZDownClicked(ui, id)
    alvoZ = limitar(alvoZ - 10*passo, limiteZ.min, limiteZ.max)
    atualizarCampos()
    moverGarra()
end


-- === Controle da garra ===
function onFechar()
    sim.setJointTargetPosition(juntaEsq, aberto)
    sim.setJointTargetPosition(juntaDir, -aberto)
    estadoAberto = false
    sim.addStatusbarMessage("Garra aberta.")
end

function onAbrir()
    sim.setJointTargetPosition(juntaEsq, fechado)
    sim.setJointTargetPosition(juntaDir, -fechado)
    estadoAberto = true
    sim.addStatusbarMessage("Garra fechada.")
end

-- === Funcoes auxiliares ===
function limitar(valor, minimo, maximo)
    if valor < minimo then return minimo end
    if valor > maximo then return maximo end
    return valor
end

function moverGarra()
    sim.setJointTargetPosition(juntaX, alvoX)
    sim.setJointTargetPosition(juntaY, alvoY)
    sim.setJointTargetPosition(juntaZ, alvoZ)
    sim.addStatusbarMessage(string.format("Movendo para X=%.2f, Y=%.2f, Z=%.2f", alvoX, alvoY, alvoZ))
end

function atualizarCampos()
    simUI.setEditValue(uiHandle, 1, string.format("%.2f", alvoX))
    simUI.setEditValue(uiHandle, 2, string.format("%.2f", alvoY))
    simUI.setEditValue(uiHandle, 3, string.format("%.2f", alvoZ))
end


-- === Deteccao de objeto vermelho (desativada se garra estiver fechada) ===
function detectarObjeto(img, resX, resY)
    local totalX, totalY, cont = 0, 0, 0
    for y = 0, resY - 1 do
        for x = 0, resX - 1 do
            local i = 3 * (y * resX + x)
            local r = string.byte(img, i + 1)
            local g = string.byte(img, i + 2)
            local b = string.byte(img, i + 3)
            if r and g and b and r > 150 and g < 80 and b < 80 then
                totalX = totalX + x
                totalY = totalY + y
                cont = cont + 1
            end
        end
    end
    if cont > 50 then
        return totalX / cont, totalY / cont
    end
    return nil, nil
end


-- === Atualiza a imagem e detecta o objeto ===
function sysCall_sensing()
    if camera and uiHandle and estadoAberto then
        local img, resX, resY = sim.getVisionSensorCharImage(camera)
        if img then
            simUI.setImageData(uiHandle, 10, img, resX, resY)
            local px, py = detectarObjeto(img, resX, resY)
            if px and py then
                local dx = (px - resX / 2) / resX
                local dy = (py - resY / 2) / resY
                alvoX = limitar(alvoX + dx * 0.05, limiteX.min, limiteX.max)
                alvoY = limitar(alvoY - dy * 0.05, limiteY.min, limiteY.max)
                moverGarra()
            end
        end
    end
end


-- === Encerramento ===
function sysCall_cleanup()
    if uiHandle then
        simUI.destroy(uiHandle)
    end
end
