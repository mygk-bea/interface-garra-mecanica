import os
import tkinter as tk
import time
import numpy as np
import threading # Necessário para evitar travamento da GUI durante movimentos longos

# Importa os módulos
import comunicacao as COM
import cinematica as C
import interface as GUI
import alfabeto as A # NOVO: Importa o módulo de fonte vetorial

# =========================
# Variáveis de Estado Global
# =========================
# Variáveis de posição atual (em radianos)
theta1_atual = 0.0
theta2_atual = 0.0
theta3_atual = 0.0
theta4_atual = 0.0
widgets = {} # Dicionário para armazenar referências de widgets
root = None  # Referência à janela principal
Z_REF_PAPEL = C.Lbase # Posição vertical inicial que representa o plano de escrita
Z_SAFE_ALTURA = 2.0  # Altura de segurança acima do plano de escrita (cm)


# =========================
# Funções Auxiliares
# =========================
def log_mensagem(mensagem, level="INFO"):
    """Função auxiliar para padronizar o log na interface."""
    if "log_text" in widgets:
        log_text = widgets["log_text"]
        log_text.insert(tk.END, f"[{level}] {mensagem}\n")
        log_text.see(tk.END)
    # Apenas para garantir que mensagens iniciais sejam exibidas mesmo sem o widget
    elif COM.DEBUG: 
        print(f"[{level}] {mensagem}")
        
def atualizar_plot():
    """Desenha a posição atual do braço no gráfico 3D."""
    global theta1_atual, theta2_atual, theta3_atual, theta4_atual
    
    if "ax" not in widgets: return
    
    xs, ys, zs = C.direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
    ax = widgets["ax"]
    canvas_plot = widgets["canvas_plot"]
    
    # Lógica de plotagem idêntica à original
    ax.clear()
    ax.set_facecolor((0, 0, 0, 0))
    grid_color = (0.8, 0.8, 0.8, 0.12)
    try:
        ax.xaxis._axinfo["grid"]["color"] = grid_color
        ax.yaxis._axinfo["grid"]["color"] = grid_color
        ax.zaxis._axinfo["grid"]["color"] = grid_color
        ax.xaxis._axinfo["grid"]["linewidth"] = 0.6
        ax.yaxis._axinfo["grid"]["linewidth"] = 0.6
        ax.zaxis._axinfo["grid"]["linewidth"] = 0.6
    except Exception:
        pass
    ax.tick_params(colors=GUI.COR_TEXTO)
    ax.xaxis.label.set_color(GUI.COR_TEXTO)
    ax.yaxis.label.set_color(GUI.COR_TEXTO)
    ax.zaxis.label.set_color(GUI.COR_TEXTO)
    ax.title.set_color(GUI.COR_TEXTO)

    # Base box
    base_size, base_height = 5, C.Lbase
    xx = [-base_size, base_size, base_size, -base_size, -base_size]
    yy = [-base_size, -base_size, base_size, base_size, -base_size]
    ax.plot3D(xx, yy, np.zeros_like(xx), color='#555555', linewidth=1)
    ax.plot3D(xx, yy, np.full_like(xx, base_height), color='#555555', linewidth=1)
    for i in range(4):
        ax.plot3D([xx[i], xx[i]], [yy[i], yy[i]], [0, base_height], color='#555555', linewidth=0.9)

    # Braço e pontos
    ax.plot(xs, ys, zs, '-o', linewidth=3.2, markersize=7, color=GUI.COR_PRIMARIA)
    ax.scatter(xs[-2], ys[-2], zs[-2], color=GUI.COR_SECUNDARIA, s=100, edgecolors='#1a1a1a')
    ax.scatter(xs[-1], ys[-1], zs[-1], color=GUI.COR_ERRO, s=120, edgecolors='#1a1a1a')

    # CORREÇÃO: Fixa os limites do gráfico para evitar o movimento aparente da base.
    MAX_RANGE = 45 
    ax.set_xlim(-MAX_RANGE, MAX_RANGE)
    ax.set_ylim(-MAX_RANGE, MAX_RANGE)
    ax.set_zlim(0, MAX_RANGE) 

    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.view_init(elev=30, azim=60)
    try:
        ax.set_box_aspect([1,1,0.6])
    except Exception:
        pass
    canvas_plot.draw()

# =========================
# Funções de Controle e Movimento (Callbacks)
# =========================
def comando_motor(motor_num):
    """Controla o motor individualmente com base nas entradas da GUI."""
    try:
        # Acessa widgets via dicionário
        direcao = widgets["sentido_var_list"][motor_num].get()
        passos = widgets["passos_entry_list"][motor_num].get()
        delay = widgets["delay_entry_list"][motor_num].get()
        if passos == '': passos = '0'
        if delay == '': delay = '10'
        
        # Chama a função de comunicação
        COM.enviar_comando(motor_num + 1, direcao, int(passos), int(delay), log_mensagem)
        log_mensagem(f"Comando enviado: Motor {motor_num+1}, Dir: {direcao}, Passos: {passos}, Delay: {delay}ms", "COMANDO")
    except Exception as e:
        log_mensagem(f"Falha no Comando Motor {motor_num+1}: {e}", "ERRO")

def parar_motor(motor_num):
    """Envia um comando de parada ('P')."""
    COM.enviar_comando(motor_num + 1, 'P', 0, 10, log_mensagem)
    log_mensagem(f"Comando de PARADA enviado para o Motor {motor_num+1}", "COMANDO")

def _movimento_incremental_core(x_dest, y_dest, z_dest, passo_max=1.0):
    """Lógica principal de CI Diferencial (Incremental) - Deve rodar em thread."""
    global theta1_atual, theta2_atual, theta3_atual, theta4_atual

    xs, ys, zs = C.direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
    pos_atual = np.array([xs[-1], ys[-1], zs[-1]])
    dpos_total = np.array([x_dest, y_dest, z_dest]) - pos_atual
    distancia = np.linalg.norm(dpos_total)

    if distancia < passo_max/2:
        if "label_coord" in widgets: widgets["label_coord"].config(text=f"Posição Atual: X = {pos_atual[0]:.1f} Y = {pos_atual[1]:.1f} Z = {pos_atual[2]:.1f}")
        return

    n_passos = int(np.ceil(distancia / passo_max))
    dpos_step = dpos_total / n_passos

    log_mensagem(f"Iniciando Movimento Incremental para ({x_dest:.1f}, {y_dest:.1f}, {z_dest:.1f}) em {n_passos} sub-passos.", "DEBUG")

    for step in range(n_passos):
        J = C.calcular_jacobiano(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
        dtheta = np.linalg.pinv(J) @ dpos_step 
        
        passos_por_junta = [C.angulo_para_passos(dtheta[i], i) for i in range(4)]

        executado_radians = np.zeros(4)
        for i, p in enumerate(passos_por_junta):
            if p == 0: continue
            
            direcao = 'H' if p > 0 else 'A' 
            passos_envio = abs(p)
            
            # Comando serial
            COM.enviar_comando(i+1, direcao, passos_envio, 10, log_mensagem)
            
            # Calcula o ângulo REALMENTE executado (quantização)
            ang_exec_deg = passos_envio * C.graus_por_passo[i]
            ang_exec_rad = np.radians(ang_exec_deg)
            
            if p < 0: ang_exec_rad = -ang_exec_rad
            executado_radians[i] = ang_exec_rad

        # Atualiza o estado global
        theta1_atual += executado_radians[0]
        theta2_atual += executado_radians[1]
        theta3_atual += executado_radians[2]
        theta4_atual += executado_radians[3]
        
        # Atualiza a GUI assincronamente
        if root: root.after(1, atualizar_plot)
    
    xs_final, ys_final, zs_final = C.direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
    pos_final = np.array([xs_final[-1], ys_final[-1], zs_final[-1]])
    
    if "label_coord" in widgets:
        widgets["label_coord"].config(text=f"Posição Atual: X = {pos_final[0]:.1f} Y = {pos_final[1]:.1f} Z = {pos_final[2]:.1f}")
    
    log_mensagem("Movimento Incremental concluído.", "INFO")


def mover_para_coordenada_seguro():
    """Callback da GUI: Inicia o movimento incremental em uma thread separada."""
    try:
        x_dest = float(widgets["entry_x"].get())
        y_dest = float(widgets["entry_y"].get())
        z_dest = float(widgets["entry_z"].get())
    except:
        log_mensagem("Coordenadas (X, Y, Z) inválidas!", "ERRO")
        return
    
    # Inicia a lógica de CI Diferencial em uma nova thread
    threading.Thread(target=_movimento_incremental_core, args=(x_dest, y_dest, z_dest)).start()

def mover_para_coordenada_seguro_interno(x_dest, y_dest, z_dest):
    """Versão síncrona/interna para uso em funções como 'escrever_texto'."""
    # Como esta função é chamada por uma thread secundária (_escrever_texto_core),
    # não precisamos de nova thread, mas garantimos a atualização da GUI.
    _movimento_incremental_core(x_dest, y_dest, z_dest)
    if root: root.update_idletasks() 

def mover_para_absoluto_fsolve(x_abs=None, y_abs=None, z_abs=None):
    """Movimento Absoluto (Cinemática Inversa com fsolve)."""
    global theta1_atual, theta2_atual, theta3_atual, theta4_atual

    if x_abs is None: # Se não forem passadas coordenadas, pega da GUI
        try:
            x_abs = float(widgets["entry_x"].get())
            y_abs = float(widgets["entry_y"].get())
            z_abs = float(widgets["entry_z"].get())
        except:
            log_mensagem("Coordenadas (X, Y, Z) inválidas!", "ERRO")
            return

    try:
        chute_inicial = [theta1_atual, theta2_atual, theta3_atual, theta4_atual]
        t1, t2, t3, t4 = C.inversa_fsolve(x_abs, y_abs, z_abs, chute_inicial)
        
        log_mensagem(f"Tentando CI para ({x_abs:.1f}, {y_abs:.1f}, {z_abs:.1f})", "DEBUG")

        deltas = [C.delta_theta(t1, theta1_atual), C.delta_theta(t2, theta2_atual), C.delta_theta(t3, theta3_atual), C.delta_theta(t4, theta4_atual)]
        dif_passos = [C.angulo_para_passos(deltas[i], i) for i in range(4)]
        
        log_mensagem(f"Movimento Absoluto. Passos: {dif_passos}", "INFO")

        for i, p in enumerate(dif_passos):
            if p == 0: continue
            direcao = 'H' if p > 0 else 'A' 
            COM.enviar_comando(i+1, direcao, abs(p), 10, log_mensagem)
        
        # Atualiza os ângulos com a solução da CI
        theta1_atual, theta2_atual, theta3_atual, theta4_atual = t1, t2, t3, t4
        atualizar_plot()
        widgets["label_coord"].config(text=f"Posição Atual: X = {x_abs:.1f} Y = {y_abs:.1f} Z = {z_abs:.1f}")
        log_mensagem("Movimento Absoluto concluído.", "INFO")

    except Exception as e:
        log_mensagem(f"Falha na Cinemática Inversa (fsolve): {e}", "ERRO")

def mover_para_home():
    """Move o braço para uma posição 'Home' (X=esticado, Y=0, Z=Base)."""
    x_home = C.L2 + C.L3 + C.L4 + C.L_final 
    y_home = 0.0
    z_home = C.Lbase 
    log_mensagem(f"Movendo para a posição HOME: ({x_home:.1f}, {y_home:.1f}, {z_home:.1f})", "COMANDO")
    
    threading.Thread(target=lambda: mover_para_absoluto_fsolve(x_abs=x_home, y_abs=y_home, z_abs=z_home)).start()


def mover_junta_temp(junta_idx):
    """Executa um movimento de teste (5 graus para frente e para trás)."""
    try:
        delta_graus=5
        delay_ms=10
        passos = int(round(delta_graus / C.graus_por_passo[junta_idx]))
        
        log_mensagem(f"Teste Junta {junta_idx+1}: {delta_graus}° Horário e Anti-horário ({passos} passos).", "TESTE")
        
        COM.enviar_comando(junta_idx+1, 'H', passos, delay_ms, log_mensagem)
        time.sleep(0.5)
        COM.enviar_comando(junta_idx+1, 'A', passos, delay_ms, log_mensagem)
        
        log_mensagem(f"Teste Junta {junta_idx+1} concluído.", "TESTE")
        
    except Exception as e:
        log_mensagem(f"Falha no Teste Junta {junta_idx+1}: {e}", "ERRO")


def escrever_texto():
    """Callback da GUI: Inicia a simulação de escrita em uma thread."""
    texto = widgets["entry_texto"].get().strip()
    try:
        escala = float(widgets["entry_scale"].get())
    except ValueError:
        log_mensagem("Escala de escrita inválida. Use um número.", "ERRO")
        return

    if not texto:
        log_mensagem("Nenhum texto para escrever.", "AVISO")
        return
        
    # Inicia a lógica de escrita em uma nova thread
    threading.Thread(target=_escrever_texto_core, args=(texto, escala)).start()

def _escrever_texto_core(texto, escala):
    """Lógica principal de escrita vetorial rodando em background."""
    log_mensagem(f"Iniciando escrita de '{texto}' com escala de {escala:.1f} cm.", "ESCRITA")

    # Pega posição inicial: Onde o braço está agora será o canto inferior esquerdo da 1ª letra.
    xs, ys, zs = C.direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
    start_x, start_y = xs[-1], ys[-1]
    
    # Alturas absolutas (Base + Z_SAFE)
    z_baixo = Z_REF_PAPEL           # Tocar papel
    z_alto = Z_REF_PAPEL + Z_SAFE_ALTURA  # Levantar
    
    # --- Sequência de Movimentos ---
    
    # 1. Levanta caneta e vai para o ponto inicial da primeira letra (no ar)
    cursor_x = start_x
    log_mensagem(f"1. Movendo para ponto inicial de segurança: ({start_x:.1f}, {start_y:.1f}, {z_alto:.1f})")
    mover_para_coordenada_seguro_interno(start_x, start_y, z_alto)

    largura_letra = escala * 1.0 
    largura_espaco = escala * 0.5 
    
    for char in texto.upper():
        strokes = A.get_char(char)
        
        if not strokes: # Caractere não mapeado (como espaço)
            cursor_x += largura_espaco
            log_mensagem(f"Avançando cursor para espaço em '{char}'.", "DEBUG")
            continue

        log_mensagem(f"Desenhando caractere: '{char}'", "DEBUG")
        
        for stroke in strokes:
            # 2. Mover (no ar) para o primeiro ponto (p0) do traço
            p0 = stroke[0]
            target_x_start = cursor_x + (p0[0] * escala)
            target_y_start = start_y + (p0[1] * escala) 
            
            # Move no ar para o ponto inicial do traço
            mover_para_coordenada_seguro_interno(target_x_start, target_y_start, z_alto) 
            
            # 3. Baixar caneta para começar o traço
            mover_para_coordenada_seguro_interno(target_x_start, target_y_start, z_baixo)
            time.sleep(0.05) 
            
            # 4. Desenhar o traço (ponto a ponto)
            for p_norm in stroke[1:]:
                next_x = cursor_x + (p_norm[0] * escala)
                next_y = start_y + (p_norm[1] * escala)
                mover_para_coordenada_seguro_interno(next_x, next_y, z_baixo) # Risca o papel
            
            # 5. Levantar ao fim do traço (volta para o último ponto riscado, mas sobe Z)
            curr_xs, curr_ys, _ = C.direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
            mover_para_coordenada_seguro_interno(curr_xs[-1], curr_ys[-1], z_alto)

        # 6. Avança o cursor para a próxima letra
        cursor_x += largura_letra + largura_espaco 

    # 7. Finaliza o movimento
    log_mensagem("Escrita concluída. Movendo para posição final de segurança.", "ESCRITA")
    mover_para_coordenada_seguro_interno(cursor_x, start_y, z_alto)
    log_mensagem("Simulação de escrita finalizada.", "ESCRITA")


# =========================
# Execução Principal
# =========================
if __name__ == "__main__":
    
    # 1. Inicializa o ambiente Tkinter e a conexão serial
    root = tk.Tk()
    root.title("CONTROLADORA DO BRAÇO MECÂNICO")
    COM.configurar_conexao(log_mensagem) # Inicializa FakeArduino se DEBUG=True

    # 2. Prepara os Callbacks
    callbacks = {
        "comando_motor": comando_motor,
        "parar_motor": parar_motor,
        "mover_junta_temp": mover_junta_temp,
        "mover_para_coordenada_seguro": mover_para_coordenada_seguro,
        "mover_para_absoluto_fsolve": lambda: threading.Thread(target=mover_para_absoluto_fsolve).start(),
        "mover_para_home": mover_para_home,
        "escrever_texto": escrever_texto,
    }
    
    # 3. Obtém posição inicial para o label
    xs_init, ys_init, zs_init = C.direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
    initial_coord = [xs_init[-1], ys_init[-1], zs_init[-1]]

    # 4. Cria a interface e armazena as referências
    widgets = GUI.criar_interface(root, callbacks, initial_coord)
    
    # 5. Desenha o plot inicial
    atualizar_plot()

    # 6. Inicia o loop principal
    root.mainloop()