import os
import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.optimize import fsolve

# =========================
# Configuração Serial Arduino
# =========================
DEBUG = True  # False para usar Arduino real

if not DEBUG:
    PORTA = "COM7"
    BAUDRATE = 9600
    arduino = serial.Serial(PORTA, BAUDRATE, timeout=1)
    time.sleep(2)
else:
    class FakeArduino:
        def write(self, data):
            try:
                log_message = f"[DEBUG] Enviando comando simulado: {data.decode().strip()}"
                log_text.insert(tk.END, log_message + "\n")
                log_text.see(tk.END)
            except Exception:
                pass
        def readline(self):
            return b"[DEBUG] resposta simulada\n"
        @property
        def in_waiting(self):
            return 0
    arduino = FakeArduino()

# =========================
# Braço Robótico (parâmetros)
# =========================
Lbase, L2, L3, L4 = 19.2, 8.0, 12.0, 8.0
L_final, Lpen = 0.0, 14.0
reducoes = [24, 24, 24, 8]
steps_per_rev_motor = 32
graus_por_passo = [360 / (steps_per_rev_motor * 64 * r) for r in reducoes]

# =========================
# Variáveis de posição atual
# =========================
theta1_atual = theta2_atual = theta3_atual = theta4_atual = 0.0

# =========================
# Funções Arduino 
# =========================
def enviar_comando(motor, direcao, passos, delay):
    cmd = f"{motor},{direcao},{passos},{delay}\n"
    arduino.write(cmd.encode())
    time.sleep(0.05)
    if DEBUG:
        try:
            log_text.insert(tk.END, f"[DEBUG] Resposta simulada do motor {motor}\n")
            log_text.see(tk.END)
        except Exception:
            pass
    else:
        while arduino.in_waiting > 0:
            log_text.insert(tk.END, arduino.readline().decode().strip() + "\n")
            log_text.see(tk.END)

def comando_motor(motor_num):
    try:
        direcao = sentido_var[motor_num].get()
        passos = passos_entry[motor_num].get()
        delay = delay_entry[motor_num].get()
        if passos == '': passos = '0'
        if delay == '': delay = '10'
        enviar_comando(motor_num + 1, direcao, int(passos), int(delay))
    except Exception as e:
        log_text.insert(tk.END, f"[ERRO] Comando Motor {motor_num+1}: {e}\n")
        log_text.see(tk.END)

def parar_motor(motor_num):
    enviar_comando(motor_num + 1, 'P', 0, 10)

# =========================
# Cinemática direta e auxiliares 
# =========================
def direta(t1, t2, t3, t4):
    phi1, phi2, phi3 = t2, t2 + t3, t2 + t3 + t4
    x1, z1 = L2*np.cos(phi1), L2*np.sin(phi1)
    x2, z2 = x1 + L3*np.cos(phi2), z1 + L3*np.sin(phi2)
    x3, z3 = x2 + L4*np.cos(phi3), z2 + L4*np.sin(phi3)
    x_base, z_base = x3 + L_final*np.cos(phi3), z3 + L_final*np.sin(phi3)
    x_pen, z_pen = x_base + Lpen*np.sin(phi3), z_base - Lpen*np.cos(phi3)
    xs_local = np.array([0, x1, x2, x3, x_base, x_pen])
    zs_local = np.array([0, z1, z2, z3, z_base, z_pen])
    ys_local = np.zeros_like(xs_local)
    xs = xs_local*np.cos(t1) - ys_local*np.sin(t1)
    ys = xs_local*np.sin(t1) + ys_local*np.cos(t1)
    zs = Lbase + zs_local
    return xs, ys, zs

def angulo_para_passos(delta_rad, motor_idx):
    return int(round(np.degrees(delta_rad) / graus_por_passo[motor_idx]))

def delta_theta(theta_dest, theta_atual):
    d = theta_dest - theta_atual
    while d > np.pi: d -= 2*np.pi
    while d < -np.pi: d += 2*np.pi
    return d

# =========================
# Função de erro para fsolve
# =========================
def erro_angulos(thetas, x_desejado, y_desejado, z_desejado):
    t1, t2, t3, t4 = thetas
    xs, ys, zs = direta(t1, t2, t3, t4)
    x_atual, y_atual, z_atual = xs[-1], ys[-1], zs[-1]
    return [
        x_atual - x_desejado,
        y_atual - y_desejado,
        z_atual - z_desejado,
        t2 + t3 + t4
    ]

def inversa_fsolve(x, y, z, chute_inicial=None):
    global theta1_atual, theta2_atual, theta3_atual, theta4_atual
    if chute_inicial is None:
        chute_inicial = [theta1_atual, theta2_atual, theta3_atual, theta4_atual]
    sol = fsolve(erro_angulos, chute_inicial, args=(x, y, z))
    return sol

# =========================
# Movimento incremental 
# =========================
def calcular_jacobiano(t1, t2, t3, t4, delta=1e-6):
    f0 = np.array(direta(t1, t2, t3, t4))
    pos0 = np.array([f0[0,-1], f0[1,-1], f0[2,-1]])
    J = np.zeros((3,4))
    thetas = [t1, t2, t3, t4]
    for i in range(4):
        dtheta = np.zeros(4)
        dtheta[i] = delta
        f1 = np.array(direta(*(thetas[j]+dtheta[j] for j in range(4))))
        pos1 = np.array([f1[0,-1], f1[1,-1], f1[2,-1]])
        J[:,i] = (pos1 - pos0)/delta
    return J

def mover_para_coordenada_seguro():
    global theta1_atual, theta2_atual, theta3_atual, theta4_atual
    try:
        x_dest = float(entry_x.get())
        y_dest = float(entry_y.get())
        z_dest = float(entry_z.get())
    except:
        log_text.insert(tk.END, "[ERRO] Coordenadas inválidas!\n")
        log_text.see(tk.END)
        return

    xs, ys, zs = direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
    pos_atual = np.array([xs[-1], ys[-1], zs[-1]])
    dpos_total = np.array([x_dest, y_dest, z_dest]) - pos_atual
    distancia = np.linalg.norm(dpos_total)
    passo_max = 1.0
    n_passos = int(np.ceil(distancia / passo_max))
    if n_passos == 0:
        label_coord.config(text=f"Posição Atual: X = {pos_atual[0]:.1f} Y = {pos_atual[1]:.1f} Z = {pos_atual[2]:.1f}")
        return
    dpos_step = dpos_total / n_passos

    log_text.insert(tk.END, f"[INFO] Movimento Incremental para ({x_dest:.1f}, {y_dest:.1f}, {z_dest:.1f}) em {n_passos} passos.\n")
    log_text.see(tk.END)

    for _ in range(n_passos):
        J = calcular_jacobiano(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
        dtheta = np.linalg.pinv(J) @ dpos_step
        dif_passos = [int(round(np.degrees(dtheta[i]) / graus_por_passo[i])) for i in range(4)]
        for i, p in enumerate(dif_passos):
            if p == 0: continue
            direcao = 'H' if p < 0 else 'A'
            enviar_comando(i+1, direcao, abs(p), 10)
        theta1_atual += dtheta[0]
        theta2_atual += dtheta[1]
        theta3_atual += dtheta[2]
        theta4_atual += dtheta[3]
        atualizar_plot()
    
    label_coord.config(text=f"Posição Atual: X = {x_dest:.1f} Y = {y_dest:.1f} Z = {z_dest:.1f}")
    log_text.insert(tk.END, "[INFO] Movimento Incremental concluído.\n")
    log_text.see(tk.END)

# =========================
# Movimento absoluto com fsolve
# =========================
def mover_para_absoluto_fsolve():
    global theta1_atual, theta2_atual, theta3_atual, theta4_atual
    try:
        x_abs = float(entry_x.get())
        y_abs = float(entry_y.get())
        z_abs = float(entry_z.get())
    except:
        log_text.insert(tk.END, "[ERRO] Coordenadas inválidas!\n")
        log_text.see(tk.END)
        return

    try:
        t1, t2, t3, t4 = inversa_fsolve(x_abs, y_abs, z_abs)
        deltas = [delta_theta(t1, theta1_atual),
                  delta_theta(t2, theta2_atual),
                  delta_theta(t3, theta3_atual),
                  delta_theta(t4, theta4_atual)]
        dif_passos = [angulo_para_passos(deltas[i], i) for i in range(4)]
        
        log_text.insert(tk.END, f"[INFO] Movimento Absoluto para ({x_abs:.1f}, {y_abs:.1f}, {z_abs:.1f}).\n")
        log_text.see(tk.END)

        for i, p in enumerate(dif_passos):
            if p == 0: continue
            direcao = 'H' if p > 0 else 'A'
            enviar_comando(i+1, direcao, abs(p), 10)
        
        theta1_atual, theta2_atual, theta3_atual, theta4_atual = t1, t2, t3, t4
        atualizar_plot()
        label_coord.config(text=f"Posição Atual: X = {x_abs:.1f} Y = {y_abs:.1f} Z = {z_abs:.1f}")
        log_text.insert(tk.END, "[INFO] Movimento Absoluto concluído.\n")
        log_text.see(tk.END)

    except Exception as e:
        log_text.insert(tk.END, f"[ERRO] Falha na Cinemática Inversa (fsolve): {e}\n")
        log_text.see(tk.END)

# =========================
# Movimento juntas
# =========================
def mover_junta_temp(junta_idx):
    try:
        delta_graus=5
        delay_ms=10
        passos = int(round(delta_graus / graus_por_passo[junta_idx]))
        log_text.insert(tk.END, f"[INFO] Teste Junta {junta_idx+1}: {delta_graus}° Horário e Anti-horário.\n")
        log_text.see(tk.END)
        enviar_comando(junta_idx+1, 'H', passos, delay_ms)
        enviar_comando(junta_idx+1, 'A', passos, delay_ms)
        log_text.insert(tk.END, f"[INFO] Teste Junta {junta_idx+1} concluído.\n")
        log_text.see(tk.END)
    except Exception as e:
        log_text.insert(tk.END, f"[ERRO] Teste Junta {junta_idx+1}: {e}\n")
        log_text.see(tk.END)

# =========================
# Envio de texto 
# =========================
def enviar_texto():
    texto = entry_texto.get()
    if texto:
        log_text.insert(tk.END, f"[COMANDO] Texto enviado: {texto}\n")
        log_text.see(tk.END)
        entry_texto.delete(0, tk.END)

# =========================
# Interface
# =========================
root = tk.Tk()
root.title("CONTROLADORA DO BRAÇO MECÂNICO")

root.state("zoomed") 

base_path = os.path.dirname(os.path.abspath(__file__))
theme_path = os.path.join(base_path, "theme", "custom_dark.tcl")

style = ttk.Style()
if os.path.exists(theme_path):
    try:
        root.tk.call('source', theme_path)
    except Exception:
        pass

# Paleta
COR_PRIMARIA = '#188181'
COR_SECUNDARIA = '#FFD166'
COR_ERRO = '#b94444'
COR_FUNDO = '#181818'
COR_FUNDO_WIDGET = '#222222'
COR_TEXTO = '#EDEDED'
INPUT_BG = '#151515'
COR_BORDA = '#2F2F2F'
DISABLED_BG = '#555555'

container = tk.Frame(root, bg=COR_FUNDO)
container.pack(fill="both", expand=True)

canvas = tk.Canvas(
    container,
    bg=COR_FUNDO,
    highlightthickness=0
)
canvas.pack(side="left", fill="both", expand=True)

scrollbar = tk.Scrollbar(
    container,
    orient="vertical",
    command=canvas.yview
)
scrollbar.pack(side="right", fill="y")

canvas.configure(yscrollcommand=scrollbar.set)

# Frame que segura TODO o conteúdo
content_frame = tk.Frame(canvas, bg=COR_FUNDO)

# cria uma window dentro do canvas
canvas_window = canvas.create_window((0, 0), window=content_frame, anchor="nw")


def resize_content(event):
    # Faz o content_frame ter exatamente a largura do canvas
    canvas.itemconfig(canvas_window, width=event.width)

# Atualiza scrollregion quando os widgets mudarem de tamanho
def update_scroll(event):
    canvas.configure(scrollregion=canvas.bbox("all"))

content_frame.bind("<Configure>", update_scroll)
canvas.bind("<Configure>", resize_content)


# Suporte a rolagem pelo mouse
def _on_mousewheel(event):
    canvas.yview_scroll(int(-1*(event.delta/120)), "units")

canvas.bind_all("<MouseWheel>", _on_mousewheel)

default_font = ("Segoe UI", 10)
style.configure(".", background=COR_FUNDO, foreground=COR_TEXTO, font=default_font)
style.configure("Card.TLabelframe", background=COR_FUNDO_WIDGET, foreground=COR_PRIMARIA, borderwidth=0)
style.configure("Card.TLabelframe.Label", background=COR_FUNDO_WIDGET, foreground=COR_PRIMARIA, font=("Segoe UI", 11, "bold"))


for i in range(5):
    content_frame.grid_columnconfigure(i, weight=1)
content_frame.grid_columnconfigure(4, weight=2)
for r in range(7):
    content_frame.grid_rowconfigure(r, weight=0)
content_frame.grid_rowconfigure(4, weight=1)
content_frame.grid_rowconfigure(5, weight=0)
content_frame.grid_rowconfigure(6, weight=0)

def decorative_label(master, text, fg=COR_PRIMARIA, bg=COR_FUNDO, height=3, pad_top=6, pad_bottom=6):
    wrap = tk.Frame(master, bg=bg)
    c = tk.Canvas(wrap, height=height+pad_top+pad_bottom, bg=bg, highlightthickness=0)
    c.pack(fill='x', expand=True)
    font = ("Segoe UI", 11, "bold")
    tkfont_obj = tkfont.Font(font=font)
    def redraw(event=None):
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        y = h // 2
        txt_w = tkfont_obj.measure(text)
        pad = 18
        left_x2 = max(12, (w - txt_w)//2 - pad)
        right_x1 = min(w-12, (w + txt_w)//2 + pad)
        line_thickness = 3
        c.create_line(8, y, left_x2, y, width=line_thickness, fill=fg)
        c.create_line(right_x1, y, w-8, y, width=line_thickness, fill=fg)
        c.create_rectangle(left_x2-8, y-16, right_x1+8, y+16, fill=bg, outline=bg)
        c.create_text(w//2, y, text=text, fill=COR_TEXTO, font=font)
    c.bind("<Configure>", redraw)
    return wrap

# ---------- Título principal ----------
title_lbl = tk.Label(content_frame, text="CONTROLADORA DO BRAÇO MECÂNICO",
font=("Segoe UI", 26, "bold"), fg=COR_PRIMARIA, bg=COR_FUNDO)
title_lbl.grid(row=0, column=0, columnspan=5, pady=(22, 12), padx=(20), sticky='w')

# ---------- 1. Motores Individuais (2x2 cards) ----------
frame_motores = tk.Frame(content_frame, bg=COR_FUNDO)
frame_motores.grid(row=1, column=0, columnspan=4, sticky='nsew', padx=18, pady=(8,10))
decor = decorative_label(frame_motores, "Motores Individuais")
decor.pack(fill='x', pady=(0,10))

inner_motors = tk.Frame(frame_motores, bg=COR_FUNDO)
inner_motors.pack(fill='both', expand=True)
for c in range(2):
    inner_motors.columnconfigure(c, weight=1)

sentido_var, passos_entry, delay_entry = [], [], []
for i in range(4):
    card = tk.Frame(inner_motors, bg=COR_FUNDO_WIDGET, bd=0)
    card.grid(row=i//2, column=i%2, padx=10, pady=10, sticky='nsew')
    card.configure(height=180)
    card.grid_propagate(False)

    lbl = tk.Label(card, text=f"Motor {i+1}", bg=COR_FUNDO_WIDGET, fg=COR_TEXTO, font=("Segoe UI", 10, "bold"))
    lbl.pack(anchor='w', padx=12, pady=(10,4))

    sentido_var.append(tk.StringVar(value='H'))
    rbf = tk.Frame(card, bg=COR_FUNDO_WIDGET)
    rbf.pack(anchor='w', padx=12)
    r1 = ttk.Radiobutton(rbf, text="Horário", variable=sentido_var[i], value='H')
    r2 = ttk.Radiobutton(rbf, text="Anti-horário", variable=sentido_var[i], value='A')
    r1.pack(side='left', padx=(0,10)); r2.pack(side='left')

    lbl_pass = tk.Label(card, text="Passos:", bg=COR_FUNDO_WIDGET, fg=COR_TEXTO)
    lbl_pass.pack(anchor='w', padx=12, pady=(10,2))
    e_pass = tk.Entry(card, width=40,
    bg=INPUT_BG,         
    fg="white",        
    insertbackground="white",
    relief="flat",      
    highlightthickness=2,  
    highlightbackground="#333333", 
    highlightcolor="#333333",   
    bd=0,
    font=("Segoe UI", 10),)
    e_pass.pack(anchor='w', padx=12, ipady=5)
    passos_entry.append(e_pass)

    lbl_delay = tk.Label(card, text="Delay (ms):", bg=COR_FUNDO_WIDGET, fg=COR_TEXTO)
    lbl_delay.pack(anchor='w', padx=12, pady=(8,2))
    e_delay = tk.Entry(card, width=40,
    bg=INPUT_BG,          
    fg="white",           
    insertbackground="white", 
    relief="flat",        
    highlightthickness=2,   
    highlightbackground="#333333", 
    highlightcolor="#333333",    
    bd=0,
    font=("Segoe UI", 10),)
    e_delay.pack(anchor='w', padx=12,  ipady=5, pady=(0,8))
    e_delay.insert(0, "10") 
    delay_entry.append(e_delay)

    btn_f = tk.Frame(card, bg=COR_FUNDO_WIDGET)
    btn_f.pack(fill='x', padx=12, pady=(6,12))
    b_send = tk.Button(
        btn_f,
        text="Enviar",
        command=lambda m=i: comando_motor(m),
        font=("Segoe UI", 10, "bold"),
        bg=COR_PRIMARIA,     
        fg=COR_TEXTO,      
        activebackground="#444444",  
        activeforeground=COR_FUNDO,  
        borderwidth=0,  
        relief="flat",      
        padx=8,             
        pady=4           
    )
    b_send.pack(padx=5, pady=5)
    b_stop = tk.Button(btn_f, text="Parar", command=lambda m=i: parar_motor(m),
    font=("Segoe UI", 10, "bold"),
    bg=COR_ERRO,             
    fg=COR_TEXTO,      
    activebackground="#C9302C", 
    activeforeground=COR_FUNDO,
    borderwidth=0,
    relief="flat",
    padx=8,                  
    pady=4                    
    )
    b_send.pack(side='left', expand=True, fill='x', padx=(0,8))
    b_stop.pack(side='left', expand=True, fill='x')

# ---------- 2. Executar Testes (linha de botões) ----------
frame_testes = tk.Frame(content_frame, bg=COR_FUNDO)
frame_testes.grid(row=2, column=0, columnspan=4, sticky='ew', padx=18, pady=(4,8))
decor_testes = decorative_label(frame_testes, "Executar Testes (5° por junta)")
decor_testes.pack(fill='x', pady=(0,15))
btns_testes_frame = tk.Frame(frame_testes, bg=COR_FUNDO)
btns_testes_frame.pack(fill='x', padx=8)
for i in range(4):
    b = tk.Button(btns_testes_frame, text=f"Junta {i+1}", command=lambda j=i: mover_junta_temp(j),   
        font=("Segoe UI", 10, "bold"),
        bg=COR_PRIMARIA,  
        fg=COR_TEXTO,    
        activebackground="#444444",
        activeforeground=COR_FUNDO,
        borderwidth=0,   
        relief="flat",   
        padx=8,             
        pady=4               
        )
    b.pack(side='left', expand=True, fill='x', padx=8)

# ---------- 3. Digite um texto ----------
frame_texto = tk.Frame(content_frame, bg=COR_FUNDO)
frame_texto.grid(row=3, column=0, columnspan=4, sticky='ew', padx=18, pady=(4,8))
decor_texto = decorative_label(frame_texto, "Envio de Texto")
decor_texto.pack(fill='x',  pady=(10, 10))
txt_inner = tk.Frame(frame_texto, bg=COR_FUNDO)
txt_inner.pack(fill='x', padx=10)
entry_texto = tk.Entry(txt_inner,
    bg=INPUT_BG,       
    fg="white",         
    insertbackground="white", 
    relief="flat",         
    highlightthickness=2,  
    highlightbackground="#333333", 
    highlightcolor="#333333",    
    bd=0,
    font=("Segoe UI", 10),)
entry_texto.pack(side='left', fill='x', expand=True, padx=(0,10), ipady=5, pady=(4, 0))
tk.Button(txt_inner, text="Enviar", 
        command=enviar_texto,   
        font=("Segoe UI", 10, "bold"),
        bg=COR_PRIMARIA,     
        fg=COR_TEXTO,  
        activebackground="#444444",  
        activeforeground=COR_FUNDO, 
        borderwidth=0,       
        relief="flat",   
        padx=8,              
        pady=4               
        ).pack(side='right')

# ---------- 4. Log ----------
frame_log = tk.Frame(content_frame, bg=COR_FUNDO)

frame_log.grid(row=4, column=0, columnspan=4, sticky='nsew', padx=18, pady=(5,10))
decor_log = decorative_label(frame_log, "Log")
decor_log.pack(fill='x', pady=(0,8))
log_box = tk.Frame(frame_log, bg=COR_FUNDO)
log_box.pack(fill='both', expand=True, padx=10)
log_text = tk.Text(log_box, height=5, bg=COR_FUNDO_WIDGET, fg=COR_TEXTO, insertbackground=COR_TEXTO, relief=tk.FLAT)
log_text.pack(fill='both', expand=True)

# ---------- 5. Painel 3D (direita) ----------
frame_plot = tk.Frame(content_frame, bg=COR_FUNDO_WIDGET)
frame_plot.grid(row=1, column=4, rowspan=4, sticky='nsew', padx=18, pady=12)
frame_plot.grid_rowconfigure(0, weight=1)
frame_plot.grid_columnconfigure(0, weight=1)
plot_decor = decorative_label(frame_plot, "Modelo Tridimensional")
plot_decor.pack(fill='x', pady=(0,10))

fig = plt.figure(figsize=(10,5)) 
fig.patch.set_facecolor(COR_FUNDO_WIDGET)
ax = fig.add_subplot(111, projection='3d')
plot_canvas  = FigureCanvasTkAgg(fig, master=frame_plot)
plot_canvas.get_tk_widget().pack(fill='both', expand=True, padx=12, pady=(0,8))

xs_init, ys_init, zs_init = direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
label_coord = tk.Label(frame_plot, text=f"Posição Atual: X = {xs_init[-1]:.1f} Y = {ys_init[-1]:.1f} Z = {zs_init[-1]:.1f}",
bg=COR_FUNDO_WIDGET, fg=COR_TEXTO)
label_coord.pack(pady=(0,20))

# ---------- 6. Coordenadas e botões de movimento (rodapé) ----------
frame_coord = tk.Frame(content_frame, bg=COR_FUNDO)
frame_coord.grid(row=5, column=0, columnspan=5, sticky='ew', padx=18, pady=(0,18))
decor_coord = decorative_label(frame_coord, "Definir Coordenadas")
decor_coord.pack(fill='x', pady=(0,10))
coord_inner = tk.Frame(frame_coord, bg=COR_FUNDO)
coord_inner.pack(fill='x', padx=10)

# grid columns
for c in range(8):
    coord_inner.columnconfigure(c, weight=0)
coord_inner.columnconfigure(1, weight=1)
coord_inner.columnconfigure(3, weight=1)
coord_inner.columnconfigure(5, weight=1)

# entries e botões de coordenadas
tk.Label(coord_inner, text="X:", bg=COR_FUNDO, fg=COR_TEXTO).grid(row=0, column=0, padx=(0,6), sticky='e')
entry_x = tk.Entry(coord_inner, width=20,
    bg=INPUT_BG,         
    fg="white",          
    insertbackground="white",
    relief="flat",   
    highlightthickness=2, 
    highlightbackground="#333333",
    highlightcolor="#333333",      
    bd=0,
    font=("Segoe UI", 10),); entry_x.grid(row=0, column=1, padx=(0,12), sticky='w', pady=5, ipady=4)
tk.Label(coord_inner, text="Y:", bg=COR_FUNDO, fg=COR_TEXTO).grid(row=0, column=2, padx=(0,6), sticky='e')
entry_y = tk.Entry(coord_inner, width=20,
    bg=INPUT_BG,          
    fg="white",            
    insertbackground="white", 
    relief="flat",        
    highlightthickness=2,   
    highlightbackground="#333333", 
    highlightcolor="#333333",      
    bd=0,
    font=("Segoe UI", 10),); entry_y.grid(row=0, column=3, padx=(0,12), sticky='w', pady=5, ipady=4,)
tk.Label(coord_inner, text="Z:", bg=COR_FUNDO, fg=COR_TEXTO).grid(row=0, column=4, padx=(0,6), sticky='e')
entry_z = tk.Entry(coord_inner, width=20,
    bg=INPUT_BG,          
    fg="white",            
    insertbackground="white", 
    relief="flat",          
    highlightthickness=2,   
    highlightbackground="#333333", 
    highlightcolor="#333333",     
    bd=0,
    font=("Segoe UI", 10),); entry_z.grid(row=0, column=5, padx=(0,12), pady=5, ipady=4, sticky='w')

tk.Button(coord_inner, text="Movimento Incremental", 
        command=mover_para_coordenada_seguro,   
        font=("Segoe UI", 10, "bold"),
        bg=COR_PRIMARIA,      
        fg=COR_TEXTO,        
        activebackground="#444444", 
        activeforeground=COR_FUNDO,  
        borderwidth=0,       
        relief="flat",        
        padx=8,               
        pady=4                
).grid(row=0, column=8, padx=8)

tk.Button(coord_inner, text="Movimento Absoluto (fsolve)", 
        command=mover_para_absoluto_fsolve,   
        font=("Segoe UI", 10, "bold"),
        bg=COR_PRIMARIA,      
        fg=COR_TEXTO,         
        activebackground="#444444",  
        activeforeground=COR_FUNDO, 
        borderwidth=0,        
        relief="flat",        
        padx=8,               
        pady=4                
).grid(row=0, column=9, padx=(20,60))

tk.Button(coord_inner, text="Home", 
        command=mover_para_absoluto_fsolve,   
        font=("Segoe UI", 10, "bold"),
        bg=COR_ERRO,      
        fg=COR_TEXTO,         
        activebackground="#444444",  
        activeforeground=COR_FUNDO, 
        borderwidth=0,        
        relief="flat",        
        padx=8,               
        pady=4                
).grid(row=0, column=10)

# ---------- Função de desenho do plot ----------
def atualizar_plot():
    xs, ys, zs = direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
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
    ax.tick_params(colors=COR_TEXTO)
    ax.xaxis.label.set_color(COR_TEXTO)
    ax.yaxis.label.set_color(COR_TEXTO)
    ax.zaxis.label.set_color(COR_TEXTO)
    ax.title.set_color(COR_TEXTO)

    # Base box
    base_size, base_height = 5, Lbase
    xx = [-base_size, base_size, base_size, -base_size, -base_size]
    yy = [-base_size, -base_size, base_size, base_size, -base_size]
    ax.plot3D(xx, yy, np.zeros_like(xx), color='#555555', linewidth=1)
    ax.plot3D(xx, yy, np.full_like(xx, base_height), color='#555555', linewidth=1)
    for i in range(4):
        ax.plot3D([xx[i], xx[i]], [yy[i], yy[i]], [0, base_height], color='#555555', linewidth=0.9)

    # Braço e pontos
    ax.plot(xs, ys, zs, '-o', linewidth=3.2, markersize=7, color=COR_PRIMARIA)
    ax.scatter(xs[-2], ys[-2], zs[-2], color=COR_SECUNDARIA, s=100, edgecolors='#1a1a1a')
    ax.scatter(xs[-1], ys[-1], zs[-1], color=COR_ERRO, s=120, edgecolors='#1a1a1a')

    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.view_init(elev=30, azim=60)
    try:
        ax.set_box_aspect([1,1,0.6])
    except Exception:
        pass
    plot_canvas.draw()

atualizar_plot()

content_frame.mainloop() 