import os
import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont
import serial
import time
import numpy as np
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# =============================================================================
# 1. CONFIGURAÇÕES E HARDWARE
# =============================================================================

DEBUG = False # False = MANDA PRO ARDUINO REAL
PORTA_SERIAL = "COM13"
BAUDRATE = 9600

# Mapeamento de Portas Físicas (Usamos apenas o Eixo 1 e Eixo 2 lógicos)
# Eixo 1 (Horizontal/Base) -> Motor 1 (Índice Lógico 0)
# Eixo 2 (Vertical/Altura) -> Motor 4 (Índice Lógico 1)
MAP_MOTORES_FISICOS = [1, 4] 

DELAY_EIXO_1 = 3
DELAY_EIXO_2 = 3

REDUCOES = [24, 24] 
STEPS_PER_REV_MOTOR = 32
MICROSTEPS = 128 
GRAUS_POR_PASSO = [360 / (STEPS_PER_REV_MOTOR * MICROSTEPS * r) for r in REDUCOES]

FATOR_CORRECAO = [1, 1]

# Geometria do Robô (cm)
L_BASE = 18.0
L2, L3, L4, L_PEN = 10.0, 11.0, 13.5, 11.0
R_ARM = L2 + L3 + L4 + L_PEN 

RESOLUCAO_LINHA_CM = 0.2

# Estado Global
theta1_atual = 0.0
theta2_atual = 0.0
arduino = None
root = None
widgets = {} # Armazena os widgets da nova interface

# =============================================================================
# 2. COMUNICAÇÃO SERIAL (Sem alterações)
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
    
    if FATOR_CORRECAO[eixo_logico] < 0:
        direcao = 'A' if direcao == 'H' else 'H'
        
    cmd = f"{motor_fisico},{direcao},{passos},{delay}\n"
    arduino.write(cmd.encode())
    
    time.sleep(0.002) 
    
    if not DEBUG:
        while arduino.in_waiting > 0: arduino.readline()

# =============================================================================
# 3. CINEMÁTICA E INTERPOLAÇÃO (Mantida a função mover_linear)
# =============================================================================
def cinematica_direta(t1, t2):
    """Calcula posição (X, Y, Z). X é a profundidade."""
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
            # Eixo 1 (Lógico 0) -> Motor 1 físico
            if p1 != 0:
                enviar_comando_fisico(0, 'H' if p1 > 0 else 'A', abs(p1), DELAY_EIXO_1)
            # Eixo 2 (Lógico 1) -> Motor 4 físico
            if p2 != 0:
                enviar_comando_fisico(1, 'H' if p2 > 0 else 'A', abs(p2), DELAY_EIXO_2)
                
            theta1_atual += np.radians(p1 * GRAUS_POR_PASSO[0])
            theta2_atual += np.radians(p2 * GRAUS_POR_PASSO[1])
            
            if i % 10 == 0 and root: root.update()

    if root:
        root.after(1, atualizar_plot)
        atualizar_label()

# =============================================================================
# 4. ALFABETO E THREAD DE ESCRITA (Sem alterações na lógica)
# =============================================================================
FONT = {
    'L': [[(0.0, 1.0), (0.0, 0.0), (1.0, 0.0)]],
    'V': [[(0.0, 1.0), (0.5, 0.0), (1.0, 1.0)]],
    ' ': []
}

def get_char(c):
    return FONT.get(c.upper(), [])

def thread_escrever(texto, escala):
    log(f"Escrevendo '{texto}' (Escala {escala}cm)...", "ESCRITA")
    
    _, cur_y, cur_z = cinematica_direta(theta1_atual, theta2_atual)
    z_base = cur_z 
    mover_linear(cur_y, cur_z)
    
    for char in texto.upper():
        strokes = get_char(char)
        
        if not strokes: # Espaço
            cur_y += escala * 0.6
            continue
            
        for stroke in strokes:
            p0 = stroke[0]
            y_start = cur_y + (p0[0] * escala)
            z_start = z_base + (p0[1] * escala)
            
            mover_linear(y_start, z_start)
            
            for p in stroke[1:]:
                y_target = cur_y + (p[0] * escala)
                z_target = z_base + (p[1] * escala)
                mover_linear(y_target, z_target)
        
        cur_y += escala * 1.2
        
    log("Escrita concluída.", "SUCESSO")

# =============================================================================
# 5. FUNÇÕES DE CALLBACKS E ATUALIZAÇÃO DA INTERFACE
# =============================================================================
def log(msg, lvl="INFO"):
    print(f"[{lvl}] {msg}")
    if 'log_text' in widgets:
        widgets['log_text'].insert(tk.END, f"[{lvl}] {msg}\n")
        widgets['log_text'].see(tk.END)

def atualizar_label():
    x, y, z = cinematica_direta(theta1_atual, theta2_atual)
    widgets['label_coord'].config(text=f"Y (Lat) = {y:.1f} cm | Z (Alt) = {z:.1f} cm")

def atualizar_plot():
    ax = widgets['ax']
    ax.clear()
    xe, ye, ze = cinematica_direta(theta1_atual, theta2_atual)
    
    # Braço (Base (0,0,0) -> L_BASE (0,0, L_BASE) -> Ponto Final (xe, ye, ze))
    ax.plot([0, xe], [0, ye], [L_BASE, ze], '-o', lw=4, c=COR_PRIMARIA)
    ax.plot([0, 0], [0, 0], [0, L_BASE], '--', lw=4, c=DISABLED_BG) 
    
    # Posição Final (Referência)
    ax.scatter(xe, ye, ze, marker='o', s=50, c=COR_ERRO)

    # Configurações do Plot
    ax.set_xlim(-50,50); ax.set_ylim(-50,50); ax.set_zlim(0,80)
    ax.set_xlabel('Profundidade (X)'); ax.set_ylabel('Parede (Y)'); ax.set_zlabel('Altura (Z)')
    
    # Configurar cores escuras para o plot 3D
    ax.set_facecolor(COR_FUNDO_WIDGET)
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    ax.xaxis.set_pane_color((0.1, 0.1, 0.1, 1.0))
    ax.yaxis.set_pane_color((0.1, 0.1, 0.1, 1.0))
    ax.zaxis.set_pane_color((0.1, 0.1, 0.1, 1.0))
    ax.tick_params(colors=COR_TEXTO)
    ax.xaxis.label.set_color(COR_TEXTO)
    ax.yaxis.label.set_color(COR_TEXTO)
    ax.zaxis.label.set_color(COR_TEXTO)

    widgets['canvas_plot'].draw()

def manual_action_callback(eixo_logico):
    """
    Move o Eixo 1 (lógico 0) ou Eixo 2 (lógico 1).
    """
    try:
        d = widgets[f'dir_{eixo_logico}'].get()
        p = int(widgets[f'pas_{eixo_logico}'].get())
        # Lendo o campo de Delay/Speed
        delay_val = int(widgets[f'dly_{eixo_logico}'].get())
        
        enviar_comando_fisico(eixo_logico, d, p, delay_val)
        
        global theta1_atual, theta2_atual
        sinal = 1 if d == 'H' else -1
        delta = np.radians(p * GRAUS_POR_PASSO[eixo_logico]) * sinal
        if eixo_logico == 0: theta1_atual += delta
        else: theta2_atual += delta
        atualizar_plot(); atualizar_label()
        log(f"Comando Eixo {eixo_logico+1} enviado: {p} passos, Delay={delay_val}ms.", "INFO")
    except ValueError:
        log("Passos e Delay devem ser números inteiros.", "ERRO")
    except Exception as e:
        log(f"Erro ao mover motor: {e}", "ERRO")

def escrever_texto_callback():
    try:
        texto = widgets["entry_texto"].get()
        escala = float(widgets["entry_scale"].get())
        # O movimento de escrita é feito em thread para não travar a GUI
        threading.Thread(target=thread_escrever, args=(texto, escala)).start()
    except ValueError:
        log("Escala deve ser um número válido (ex: 10.0).", "ERRO")
    
# Definindo os callbacks que serão usados
CALLBACKS = {
    "comando_eixo0": lambda: manual_action_callback(0),
    "comando_eixo1": lambda: manual_action_callback(1),
    "escrever_texto": escrever_texto_callback,
}

# =============================================================================
# 6. INTERFACE GRÁFICA (Estrutura do Original + Estilo Dark)
# =============================================================================
# --- Constantes de Estilo ---
COR_PRIMARIA = '#188181'
COR_SECUNDARIA = '#FFD166'
COR_ERRO = '#b94444'
COR_FUNDO = '#181818'
COR_FUNDO_WIDGET = '#222222'
COR_TEXTO = '#EDEDED'
INPUT_BG = '#151515'
DISABLED_BG = '#555555'

def criar_gui():
    global root, widgets
    root = tk.Tk(); root.title("Robô Parede - Controle")
    root.state("zoomed") 
    root.configure(bg=COR_FUNDO)
    
    style = ttk.Style()
    default_font = ("Segoe UI", 10)
    style.configure(".", background=COR_FUNDO, foreground=COR_TEXTO, font=default_font)
    style.configure("TFrame", background=COR_FUNDO)
    
    # --- Layout Principal (Painel Esquerdo e Plot Direito) ---
    painel = tk.Frame(root, bg=COR_FUNDO); painel.pack(side='left', fill='y', padx=15, pady=15)
    f_plot = tk.Frame(root, bg=COR_FUNDO); f_plot.pack(side='right', fill='both', expand=True, padx=15, pady=15)

    # --- Coordenada de Posição (Topo do Plot) ---
    initial_coord = cinematica_direta(theta1_atual, theta2_atual)
    widgets['label_coord'] = tk.Label(f_plot, text=f"Y (Lat) = {initial_coord[1]:.1f} cm | Z (Alt) = {initial_coord[2]:.1f} cm", 
        font=("Segoe UI", 16, "bold"), fg=COR_TEXTO, bg=COR_FUNDO)
    widgets['label_coord'].pack(pady=10)

    # --- 3D Plot ---
    fig = plt.figure(); 
    fig.patch.set_facecolor(COR_FUNDO)
    widgets['ax'] = fig.add_subplot(111, projection='3d')
    widgets['canvas_plot'] = FigureCanvasTkAgg(fig, master=f_plot); 
    widgets['canvas_plot'].get_tk_widget().pack(fill='both', expand=True)

    # =================================================================
    # PAINEL ESQUERDO (CONTROLES)
    # =================================================================

    # --- Título Principal ---
    tk.Label(painel, text="CONTROLADORA GARRA MECÂNICA", font=("Segoe UI", 22, "bold"), fg=COR_PRIMARIA, bg=COR_FUNDO).pack(fill='x', pady=(0, 20))

    # ---------- 1. Controle Manual ----------
    tk.Label(painel, text="CONTROLE MANUAL", font=("Segoe UI", 14, "bold"), fg=COR_TEXTO, bg=COR_FUNDO).pack(fill='x', pady=(0, 5))
    
    # Card Base (Y/Y) - Eixo 0
    for i, nome, delay_val in zip(
        [0, 1], 
        ["Base (Y/Y)", "Altura (Z)"],
        [DELAY_EIXO_1, DELAY_EIXO_2]
    ):
        frm = tk.Frame(painel, bg=COR_FUNDO_WIDGET, padx=10, pady=5); frm.pack(fill='x', pady=5)
        
        tk.Label(frm, text=f"{nome}:", bg=COR_FUNDO_WIDGET, fg=COR_TEXTO).pack(side='left', padx=(0, 5))
        
        # Radio Buttons: +/-
        var = tk.StringVar(value='H'); widgets[f'dir_{i}'] = var
        rbf = tk.Frame(frm, bg=COR_FUNDO_WIDGET)
        rbf.pack(side='left', padx=(5, 5))
        ttk.Radiobutton(rbf, text="+", variable=var, value='H').pack(side='left')
        ttk.Radiobutton(rbf, text="-", variable=var, value='A').pack(side='left', padx=(5,0))
        
        # Passos
        e_p = tk.Entry(frm, width=5, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", bd=0, font=("Segoe UI", 10))
        e_p.insert(0,"200"); e_p.pack(side='left', ipady=4, padx=5)
        widgets[f'pas_{i}'] = e_p
        
        # Delay
        e_d = tk.Entry(frm, width=3, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", bd=0, font=("Segoe UI", 10))
        e_d.insert(0, str(delay_val)); e_d.pack(side='left', ipady=4, padx=(5, 10))
        widgets[f'dly_{i}'] = e_d
        
        # Botão Move
        cmd = CALLBACKS["comando_eixo0"] if i == 0 else CALLBACKS["comando_eixo1"]
        tk.Button(frm, text="MOVER", command=cmd, bg=COR_PRIMARIA, fg=COR_TEXTO, borderwidth=0, relief="flat", padx=8, pady=4).pack(side='right')
    # ---------- 2. Escrita Automática ----------
    tk.Label(painel, text="ESCRITA AUTOMÁTICA", font=("Segoe UI", 12, "bold"), fg=COR_TEXTO, bg=COR_FUNDO).pack(fill='x', pady=(20, 5))
    
    f_escrita = tk.Frame(painel, bg=COR_FUNDO_WIDGET, padx=10, pady=10); f_escrita.pack(fill='x', pady=5)
    
    tk.Label(f_escrita, text="Texto (L e V):", bg=COR_FUNDO_WIDGET, fg=COR_TEXTO).pack(anchor='w', pady=(0,2))
    entry_texto = tk.Entry(f_escrita, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", bd=0, font=("Segoe UI", 10))
    entry_texto.insert(0, "LV"); entry_texto.pack(fill='x', ipady=4, pady=(0, 10))
    widgets["entry_texto"] = entry_texto

    tk.Label(f_escrita, text="Tamanho (cm):", bg=COR_FUNDO_WIDGET, fg=COR_TEXTO).pack(anchor='w', pady=(0,2))
    entry_scale = tk.Entry(f_escrita, width=10, bg=INPUT_BG, fg="white", insertbackground="white", relief="flat", bd=0, font=("Segoe UI", 10))
    entry_scale.insert(0, "10.0"); entry_scale.pack(anchor='w', ipady=4, pady=(0, 10))
    widgets["entry_scale"] = entry_scale
    
    tk.Button(f_escrita, text="INICIAR ESCRITA", command=CALLBACKS["escrever_texto"], 
        font=("Segoe UI", 11, "bold"), bg=COR_SECUNDARIA, fg=COR_FUNDO, borderwidth=0, relief="flat", height=2).pack(fill='x', pady=(10, 0))

    # ---------- 3. Status do Sistema (Log) ----------
    tk.Label(painel, text="Status do Sistema", font=("Segoe UI", 12, "bold"), fg=COR_TEXTO, bg=COR_FUNDO).pack(fill='x', pady=(20, 5))
    
    log_text = tk.Text(painel, height=10, bg=INPUT_BG, fg=COR_TEXTO, 
                        insertbackground=COR_TEXTO, relief=tk.FLAT, borderwidth=0, wrap=tk.WORD)
    log_text.pack(fill='both', expand=True)
    widgets["log_text"] = log_text

if __name__ == "__main__":
    configurar_conexao()
    criar_gui()
    atualizar_plot()
    atualizar_label()
    root.mainloop()