import tkinter as tk
from tkinter import ttk
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.optimize import fsolve

# =========================
# Configuração Serial Arduino
# =========================
PORTA = "COM7"  # altere para sua porta (ex: "COM3" ou "/dev/ttyUSB0")
BAUDRATE = 9600
try:
    arduino = serial.Serial(PORTA, BAUDRATE, timeout=1)
    time.sleep(2)
except Exception as e:
    print(f"Erro ao abrir porta serial {PORTA}: {e}")
    arduino = None

# =========================
# Braço Robótico
# =========================
Lbase, L2, L3, L4 = 19.2, 8.0, 12.0, 8.0
L_final, Lpen = 0.0, 14.0
reducoes = [24, 24, 24, 8]
steps_per_rev_motor = 32
# graus por passo (graus por cada passo físico do sistema de redução)
graus_por_passo = [360 / (steps_per_rev_motor * 64 * r) for r in reducoes]

# =========================
# Variáveis de posição atual (radianos)
# =========================
theta1_atual = theta2_atual = theta3_atual = theta4_atual = 0.0

# =========================
# Funções Arduino
# =========================
def enviar_comando(motor, direcao, passos, delay):
    """Envia comando serial no formato 'motor,acao,passos,delay' se a serial estiver aberta."""
    if arduino is None:
        print("Arduino não conectado — comando não enviado:", motor, direcao, passos, delay)
        return
    cmd = f"{motor},{direcao},{passos},{delay}\n"
    arduino.write(cmd.encode())
    time.sleep(0.02)
    # lê resposta (se houver) para esvaziar buffer
    while arduino.in_waiting > 0:
        print(arduino.readline().decode().strip())

def comando_motor(motor_num):
    direcao = sentido_var[motor_num].get()
    passos = passos_entry[motor_num].get()
    delay = delay_entry[motor_num].get()
    if passos == '': passos = '0'
    if delay == '': delay = '10'
    enviar_comando(motor_num + 1, direcao, int(passos), int(delay))

def parar_motor(motor_num):
    enviar_comando(motor_num + 1, 'P', 0, 10)

# =========================
# Cinemática direta
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
    """Converte delta (rad) para número inteiro de passos (assina positiva/negativa)."""
    deg = np.degrees(delta_rad)
    passos = int(round(deg / graus_por_passo[motor_idx]))
    return passos

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
    # Quarta equação força a orientação (p.ex. pulso plano)
    return [
        x_atual - x_desejado,
        y_atual - y_desejado,
        z_atual - z_desejado,
        t2 + t3 + t4  # mantém pulso “plano”
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
    f0 = direta(t1, t2, t3, t4)
    pos0 = np.array([f0[0,-1], f0[1,-1], f0[2,-1]])
    J = np.zeros((3,4))
    thetas = [t1, t2, t3, t4]
    for i in range(4):
        dtheta = np.zeros(4)
        dtheta[i] = delta
        # cria lista de thetas com o delta aplicado
        thetas_pert = [thetas[j] + dtheta[j] for j in range(4)]
        f1 = direta(*thetas_pert)
        pos1 = np.array([f1[0,-1], f1[1,-1], f1[2,-1]])
        J[:,i] = (pos1 - pos0)/delta
    return J

def mover_para_coordenada_seguro():
    """
    Divide o deslocamento em pequenos passos em X,Y,Z e aplica incremental via J^+.
    Correções aplicadas:
    - usa angulo_para_passos para obter passos inteiros mantendo sinal;
    - mapeia corretamente o sinal dos passos para 'H'/'A';
    - atualiza os thetas usando o número efetivo de passos (conversão p/ radianos)
    - trata singularidade do jacobiano usando pseudo-inversa com rcond aumentado
    """
    global theta1_atual, theta2_atual, theta3_atual, theta4_atual
    try:
        x_dest = float(entry_x.get())
        y_dest = float(entry_y.get())
        z_dest = float(entry_z.get())
    except:
        print("Coordenadas inválidas!")
        return

    xs, ys, zs = direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
    pos_atual = np.array([xs[-1], ys[-1], zs[-1]])
    dpos_total = np.array([x_dest, y_dest, z_dest]) - pos_atual
    distancia = np.linalg.norm(dpos_total)
    passo_max = 1.0  # cm por iteração
    n_passos = int(np.ceil(distancia / passo_max))
    if n_passos == 0:
        label_coord.config(text=f"Pos atual: X={pos_atual[0]:.1f}, Y={pos_atual[1]:.1f}, Z={pos_atual[2]:.1f}")
        return
    dpos_step = dpos_total / n_passos

    for step_idx in range(n_passos):
        J = calcular_jacobiano(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
        # se o jacobiano for mal condicionado, aumenta rcond
        try:
            # rcond pequeno -> normal; se rank baixo, usa rcond maior
            if np.linalg.matrix_rank(J) < 3:
                dtheta = np.linalg.pinv(J, rcond=1e-2) @ dpos_step
            else:
                dtheta = np.linalg.pinv(J) @ dpos_step
        except Exception as e:
            print("Erro ao inverter J:", e)
            return

        # converte incrementos rad -> passos inteiros (preservando sinal)
        passos_signed = [angulo_para_passos(dtheta[i], i) for i in range(4)]

        # envia comandos aos motores
        for i, passos in enumerate(passos_signed):
            if passos == 0: continue
            direcao = 'H' if passos > 0 else 'A'  # padrão: positivo -> H
            enviar_comando(i+1, direcao, abs(passos), 10)

        # **Atualiza os ângulos atuais COM base nos passos aplicados**
        # (em vez de usar dtheta direto, usamos passos efetivos para ficar consistente com o que o motor executou)
        for i, passos in enumerate(passos_signed):
            # passos * graus_por_passo[i] -> graus efetivos movidos; converter para rad
            delta_deg = passos * graus_por_passo[i]
            delta_rad = np.radians(delta_deg)
            if i == 0:
                theta1_atual += delta_rad
            elif i == 1:
                theta2_atual += delta_rad
            elif i == 2:
                theta3_atual += delta_rad
            elif i == 3:
                theta4_atual += delta_rad

        # pequena espera para dar tempo ao Arduino (pode ajustar)
        # tempo estimado aproximado = passos * delay(ms) / 1000.0
        # aqui fazemos um sleep curto para não travar muito a GUI
        time.sleep(0.05)
        atualizar_plot()

    label_coord.config(text=f"Pos atual: X={x_dest:.1f}, Y={y_dest:.1f}, Z={z_dest:.1f}")

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
        print("Coordenadas inválidas!")
        return

    t1, t2, t3, t4 = inversa_fsolve(x_abs, y_abs, z_abs)
    deltas = [delta_theta(t1, theta1_atual),
              delta_theta(t2, theta2_atual),
              delta_theta(t3, theta3_atual),
              delta_theta(t4, theta4_atual)]
    dif_passos = [angulo_para_passos(deltas[i], i) for i in range(4)]
    for i, p in enumerate(dif_passos):
        if p == 0: continue
        direcao = 'H' if p > 0 else 'A'
        enviar_comando(i+1, direcao, abs(p), 10)

    # atualiza os thetas reais usando os passos efetivos enviados
    for i, p in enumerate(dif_passos):
        delta_deg = p * graus_por_passo[i]
        delta_rad = np.radians(delta_deg)
        if i == 0:
            theta1_atual += delta_rad
        elif i == 1:
            theta2_atual += delta_rad
        elif i == 2:
            theta3_atual += delta_rad
        elif i == 3:
            theta4_atual += delta_rad

    atualizar_plot()
    label_coord.config(text=f"Pos atual: X={x_abs:.1f}, Y={y_abs:.1f}, Z={z_abs:.1f}")

# =========================
# Movimento juntas de teste
# =========================
def mover_junta_temp(junta_idx, delta_graus=5, delay_ms=10, espera_s=1):
    passos = int(round(delta_graus / graus_por_passo[junta_idx]))
    enviar_comando(junta_idx+1, 'H', passos, delay_ms)
    time.sleep(espera_s)
    enviar_comando(junta_idx+1, 'A', passos, delay_ms)
    time.sleep(espera_s)

# =========================
# Plot 3D integrado
# =========================
def atualizar_plot():
    xs, ys, zs = direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
    ax.clear()
    base_size, base_height = 5, Lbase
    xx = [-base_size, base_size, base_size, -base_size, -base_size]
    yy = [-base_size, -base_size, base_size, base_size, -base_size]
    ax.plot3D(xx, yy, np.zeros_like(xx), color='gray')
    ax.plot3D(xx, yy, np.full_like(xx, base_height), color='gray')
    for i in range(4):
        ax.plot3D([xx[i], xx[i]], [yy[i], yy[i]], [0, base_height], color='gray')
    ax.plot(xs, ys, zs, '-o', linewidth=3, markersize=6)
    ax.scatter(xs[-2], ys[-2], zs[-2], color='blue', s=80)
    ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=100)
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title('Braço Robótico 4R')
    ax.view_init(elev=30, azim=60)
    ax.set_box_aspect([1,1,0.5])
    canvas.draw()

# =========================
# Interface Tkinter
# =========================
root = tk.Tk()
root.title("Controle de Motores 28BYJ-48")

sentido_var, passos_entry, delay_entry = [], [], []
for i in range(4):
    frame = ttk.LabelFrame(root, text=f"Motor {i+1}", padding=10)
    frame.grid(row=0, column=i, padx=5, pady=5)
    sentido_var.append(tk.StringVar(value='H'))
    ttk.Radiobutton(frame, text="Horario", variable=sentido_var[i], value='H').pack(anchor='w')
    ttk.Radiobutton(frame, text="Antihorario", variable=sentido_var[i], value='A').pack(anchor='w')
    ttk.Label(frame, text="Passos:").pack(anchor='w')
    e_passos = ttk.Entry(frame, width=10); e_passos.pack(anchor='w'); passos_entry.append(e_passos)
    ttk.Label(frame, text="Delay (ms):").pack(anchor='w')
    e_delay = ttk.Entry(frame, width=10); e_delay.insert(0, '10'); e_delay.pack(anchor='w'); delay_entry.append(e_delay)
    ttk.Button(frame, text="Enviar", command=lambda m=i: comando_motor(m)).pack(pady=2)
    ttk.Button(frame, text="Parar", command=lambda m=i: parar_motor(m)).pack(pady=2)

# Controle por coordenadas
frame_coord = ttk.LabelFrame(root, text="Mover por Coordenada (cm)", padding=10)
frame_coord.grid(row=1, column=0, columnspan=4, padx=5, pady=10, sticky='ew')
ttk.Label(frame_coord, text="X:").grid(row=0,column=0); entry_x = ttk.Entry(frame_coord, width=10); entry_x.grid(row=0,column=1)
ttk.Label(frame_coord, text="Y:").grid(row=0,column=2); entry_y = ttk.Entry(frame_coord, width=10); entry_y.grid(row=0,column=3)
ttk.Label(frame_coord, text="Z:").grid(row=0,column=4); entry_z = ttk.Entry(frame_coord, width=10); entry_z.grid(row=0,column=5)
ttk.Button(frame_coord, text="Mover Incremental", command=mover_para_coordenada_seguro).grid(row=0,column=6, padx=5)
ttk.Button(frame_coord, text="Mover Absoluto (fsolve)", command=mover_para_absoluto_fsolve).grid(row=0,column=7, padx=5)

# Botões de teste juntas
frame_testes = ttk.LabelFrame(root, text="Teste 5° por junta", padding=10)
frame_testes.grid(row=1, column=4, columnspan=2, padx=5, pady=10, sticky='ew')
for i in range(4):
    ttk.Button(frame_testes, text=f"Junta {i+1}", command=lambda j=i: mover_junta_temp(j)).grid(row=0, column=i, padx=5)

# Label posição atual
xs_init, ys_init, zs_init = direta(theta1_atual, theta2_atual, theta3_atual, theta4_atual)
x_init, y_init, z_init = xs_init[-1], ys_init[-1], zs_init[-1]
label_coord = ttk.Label(root, text=f"Pos atual: X={x_init:.1f}, Y={y_init:.1f}, Z={z_init:.1f}")
label_coord.grid(row=2, column=0, columnspan=4, pady=5)

# Plot 3D
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().grid(row=3, column=0, columnspan=4)
atualizar_plot()

root.mainloop()
