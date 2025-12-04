import os
import tkinter as tk
from tkinter import ttk
import serial
import time
import numpy as np
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# =============================================================================
# 1. CONFIGURAÇÕES E HARDWARE
# =============================================================================

DEBUG = False   # False = MANDA PRO ARDUINO REAL
PORTA_SERIAL = "COM13"
BAUDRATE = 9600

# Mapeamento de Portas Físicas
# Eixo 1 (Horizontal/Base) -> Motor 1
# Eixo 2 (Vertical/Altura) -> Motor 4
MAP_MOTORES_FISICOS = [1, 4] 

# --- DELAY UNIFICADO E RÁPIDO ---
DELAY_EIXO_1 = 3  # Igualado conforme solicitado
DELAY_EIXO_2 = 3  

# Ajuste de Escala (5000 passos = ~15 cm)
REDUCOES = [24, 24] 
STEPS_PER_REV_MOTOR = 32
MICROSTEPS = 128 
GRAUS_POR_PASSO = [360 / (STEPS_PER_REV_MOTOR * MICROSTEPS * r) for r in REDUCOES]

# Correção de Sentido [Eixo 1, Eixo 2] (1 = Normal, -1 = Invertido)
FATOR_CORRECAO = [1, 1]

# Geometria do Robô (cm)
L_BASE = 18.0
L2, L3, L4, L_PEN = 10.0, 11.0, 13.5, 11.0
R_ARM = L2 + L3 + L4 + L_PEN 

# Resolução da Interpolação (cm)
# Como a velocidade aumentou (delay menor), precisamos de passos suaves
RESOLUCAO_LINHA_CM = 0.2

# Estado Global
theta1_atual = 0.0
theta2_atual = 0.0
arduino = None
root = None
widgets = {}

# =============================================================================
# 2. COMUNICAÇÃO SERIAL
# =============================================================================
class FakeArduino:
    def write(self, data):
        try: time.sleep(0.001) 
        except: pass
    @property
    def in_waiting(self): return 0
    def readline(self): return b"ok\n"

def configurar_conexao():
    global arduino, DEBUG
    if not DEBUG:
        try:
            arduino = serial.Serial(PORTA_SERIAL, BAUDRATE, timeout=1)
            time.sleep(2)
            log("Conectado na Serial (COM13)!", "SUCESSO")
        except Exception as e:
            log(f"Erro Serial: {e}. Usando Simulador.", "ERRO")
            DEBUG = True
            arduino = FakeArduino()
    else:
        arduino = FakeArduino()
        log("Modo Simulação Ativado.", "INFO")

def enviar_comando_fisico(eixo_logico, direcao, passos, delay):
    if arduino is None or passos == 0: return
    
    motor_fisico = MAP_MOTORES_FISICOS[eixo_logico]
    
    # Correção de inversão
    if FATOR_CORRECAO[eixo_logico] < 0:
        direcao = 'A' if direcao == 'H' else 'H'
        
    cmd = f"{motor_fisico},{direcao},{passos},{delay}\n"
    arduino.write(cmd.encode())
    
    # Pausa minúscula para não atropelar o buffer serial do Arduino
    # Com delay de 3ms, o Arduino processa muito rápido, mas o buffer USB é gargalo
    time.sleep(0.002) 
    
    if not DEBUG:
        while arduino.in_waiting > 0: arduino.readline()

# =============================================================================
# 3. CINEMÁTICA E INTERPOLAÇÃO
# =============================================================================

def cinematica_direta(t1, t2):
    """Calcula posição (Y, Z) na parede."""
    z = L_BASE + R_ARM * np.sin(t2)
    r_proj = R_ARM * np.cos(t2)
    y = r_proj * np.sin(t1)
    x = r_proj * np.cos(t1) 
    return x, y, z

def cinematica_inversa(y, z):
    """Retorna t1, t2 para atingir (Y, Z)."""
    seno_t2 = (z - L_BASE) / R_ARM
    if abs(seno_t2) > 1.0: raise ValueError("Altura Z fora do alcance")
    t2 = np.arcsin(seno_t2)
    
    r_proj = R_ARM * np.cos(t2)
    if r_proj < 0.1: t1 = theta1_atual
    else:
        seno_t1 = y / r_proj
        if abs(seno_t1) > 1.0: raise ValueError("Lateral Y fora do alcance")
        t1 = np.arcsin(seno_t1)
    return t1, t2

def angulos_para_passos(dt1, dt2):
    p1 = int(round(np.degrees(dt1) / GRAUS_POR_PASSO[0]))
    p2 = int(round(np.degrees(dt2) / GRAUS_POR_PASSO[1]))
    return p1, p2

def mover_linear(y_dest, z_dest):
    """
    Move em LINHA RETA (Sincroniza os motores).
    Essencial quando delay é baixo (3ms).
    """
    global theta1_atual, theta2_atual
    
    _, y_atual, z_atual = cinematica_direta(theta1_atual, theta2_atual)
    
    dy = y_dest - y_atual
    dz = z_dest - z_atual
    dist = np.sqrt(dy**2 + dz**2)
    
    if dist < RESOLUCAO_LINHA_CM: num_seg = 1
    else: num_seg = int(dist / RESOLUCAO_LINHA_CM)
    
    for i in range(1, num_seg + 1):
        fracao = i / num_seg
        y_tgt = y_atual + (dy * fracao)
        z_tgt = z_atual + (dz * fracao)
        
        try:
            t1_alvo, t2_alvo = cinematica_inversa(y_tgt, z_tgt)
        except ValueError: continue
            
        dt1 = t1_alvo - theta1_atual
        dt2 = t2_alvo - theta2_atual
        
        p1, p2 = angulos_para_passos(dt1, dt2)
        
        if p1 != 0 or p2 != 0:
            # Envia Eixo 1 (3ms)
            if p1 != 0:
                enviar_comando_fisico(0, 'H' if p1 > 0 else 'A', abs(p1), DELAY_EIXO_1)
            # Envia Eixo 2 (3ms)
            if p2 != 0:
                enviar_comando_fisico(1, 'H' if p2 > 0 else 'A', abs(p2), DELAY_EIXO_2)
                
            theta1_atual += np.radians(p1 * GRAUS_POR_PASSO[0])
            theta2_atual += np.radians(p2 * GRAUS_POR_PASSO[1])
            
            # Atualiza interface a cada 10 segmentos para não travar o loop rápido
            if i % 10 == 0 and root: root.update()

    if root:
        root.after(1, atualizar_plot)
        atualizar_label()

# =============================================================================
# 4. ALFABETO: AS MAIS FÁCEIS (L e V)
# =============================================================================
FONT = {
    # L: Traço único. Baixo -> Direita. Sem arrasto.
    'L': [[(0.0, 1.0), (0.0, 0.0), (1.0, 0.0)]],

    # V: Traço único. Baixo-Diag-Centro -> Cima-Diag-Dir. Sem arrasto.
    'V': [[(0.0, 1.0), (0.5, 0.0), (1.0, 1.0)]],
    
    ' ': []
}

def get_char(c):
    return FONT.get(c.upper(), [])

def thread_escrever(texto, escala):
    log(f"Escrevendo '{texto}' (Escala {escala}cm)...", "ESCRITA")
    
    # Ponto de partida atual
    _, cur_y, cur_z = cinematica_direta(theta1_atual, theta2_atual)
    
    # Define a "linha de base" na altura atual
    z_base = cur_z 
    
    # 1. Posicionamento Inicial
    mover_linear(cur_y, cur_z)
    
    for char in texto.upper():
        strokes = get_char(char)
        
        if not strokes: # Espaço
            cur_y += escala * 0.6
            continue
            
        for stroke in strokes:
            # Traço Único Contínuo (L e V são perfeitos nisso)
            
            # Vai para o primeiro ponto (Arrastando, mas faz parte da letra se desenhado certo)
            # Em L e V, o primeiro ponto é o topo esquerdo. O robô vai riscar até lá.
            p0 = stroke[0]
            y_start = cur_y + (p0[0] * escala)
            z_start = z_base + (p0[1] * escala)
            
            # Reposicionamento (Arrasto visível)
            mover_linear(y_start, z_start)
            
            # Desenha a letra
            for p in stroke[1:]:
                y_target = cur_y + (p[0] * escala)
                z_target = z_base + (p[1] * escala)
                mover_linear(y_target, z_target)
        
        # Espaço entre letras (Arrasto horizontal visível)
        cur_y += escala * 1.2
        
    log("Escrita concluída.", "SUCESSO")

# =============================================================================
# 5. INTERFACE GRÁFICA
# =============================================================================
def log(msg, lvl="INFO"):
    print(f"[{lvl}] {msg}")
    if 'log' in widgets:
        widgets['log'].insert(tk.END, f"[{lvl}] {msg}\n")
        widgets['log'].see(tk.END)

def atualizar_label():
    x, y, z = cinematica_direta(theta1_atual, theta2_atual)
    widgets['lbl'].config(text=f"Y(Lat)={y:.1f} | Z(Alt)={z:.1f}")

def atualizar_plot():
    ax = widgets['ax']
    ax.clear()
    xe, ye, ze = cinematica_direta(theta1_atual, theta2_atual)
    # Braço
    ax.plot([0, 0, xe], [0, 0, ye], [0, L_BASE, ze], '-o', lw=4, c='#188181')
    # Parede (Referência)
    ax.plot([xe, xe], [-30, 30], [ze, ze], 'r--', alpha=0.3) 
    
    ax.set_xlim(-50,50); ax.set_ylim(-50,50); ax.set_zlim(0,80)
    ax.set_xlabel('Profundidade'); ax.set_ylabel('Parede Y'); ax.set_zlabel('Z Altura')
    widgets['canvas'].draw()

def manual_action(idx):
    try:
        d = widgets[f'dir_{idx}'].get()
        p = int(widgets[f'pas_{idx}'].get())
        dl = int(widgets[f'dly_{idx}'].get())
        enviar_comando_fisico(idx, d, p, dl)
        
        # Simulação visual simples
        global theta1_atual, theta2_atual
        sinal = 1 if d == 'H' else -1
        delta = np.radians(p * GRAUS_POR_PASSO[idx]) * sinal
        if idx == 0: theta1_atual += delta
        else: theta2_atual += delta
        atualizar_plot(); atualizar_label()
    except: pass

def criar_gui():
    global root
    root = tk.Tk(); root.title("Robô Parede - L & V (Fáceis)")
    root.state("zoomed")
    
    painel = tk.Frame(root); painel.pack(side='left', fill='y', padx=10, pady=10)
    
    # Manual
    f_man = tk.LabelFrame(painel, text="Ajuste Manual"); f_man.pack(fill='x', pady=5)
    for i, nome in enumerate(["Eixo 1 (Base)", "Eixo 2 (Vert)"]):
        frm = tk.Frame(f_man); frm.pack(fill='x', pady=2)
        tk.Label(frm, text=nome, width=12).pack(side='left')
        var = tk.StringVar(value='H'); widgets[f'dir_{i}'] = var
        tk.Radiobutton(frm, text="+", variable=var, value='H').pack(side='left')
        tk.Radiobutton(frm, text="-", variable=var, value='A').pack(side='left')
        
        e_p = tk.Entry(frm, width=5); e_p.insert(0,"200"); e_p.pack(side='left'); widgets[f'pas_{i}'] = e_p
        # Delay unificado de 3ms
        e_d = tk.Entry(frm, width=4); e_d.insert(0, str(DELAY_EIXO_1)); e_d.pack(side='left'); widgets[f'dly_{i}'] = e_d
        tk.Button(frm, text="Move", command=lambda x=i: manual_action(x)).pack(side='left')

    # Escrever
    f_txt = tk.LabelFrame(painel, text="Escrever (Foco: L e V)"); f_txt.pack(fill='x', pady=10)
    
    tk.Label(f_txt, text="Texto:").pack(anchor='w')
    etxt = tk.Entry(f_txt, font=("Arial", 12)); etxt.pack(fill='x', padx=5)
    etxt.insert(0, "LV") # Sugestão das mais fáceis
    
    tk.Label(f_txt, text="Tamanho (cm):").pack(anchor='w')
    escl = tk.Entry(f_txt, width=10); escl.insert(0, "10.0"); escl.pack(anchor='w', padx=5)
    
    tk.Button(f_txt, text="ESCREVER", bg="#FFD166", height=2,
              command=lambda: threading.Thread(target=thread_escrever, args=(etxt.get(), float(escl.get()))).start()).pack(fill='x', pady=10)

    widgets['log'] = tk.Text(painel, height=15); widgets['log'].pack(fill='both')
    
    f_plot = tk.Frame(root); f_plot.pack(side='right', fill='both', expand=True)
    widgets['lbl'] = tk.Label(f_plot, text="Pos", font=("Arial", 16)); widgets['lbl'].pack(pady=10)
    fig = plt.figure(); widgets['ax'] = fig.add_subplot(111, projection='3d')
    widgets['canvas'] = FigureCanvasTkAgg(fig, master=f_plot); widgets['canvas'].get_tk_widget().pack(fill='both', expand=True)

if __name__ == "__main__":
    configurar_conexao()
    criar_gui()
    atualizar_plot()
    atualizar_label()
    root.mainloop()