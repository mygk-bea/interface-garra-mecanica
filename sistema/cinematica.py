import numpy as np
from scipy.optimize import fsolve

Lbase, L2, L3, L4 = 19.2, 8.0, 12.0, 8.0
L_final, Lpen = 0.0, 14.0

reducoes = [24, 24, 24, 8]
steps_per_rev_motor = 32
graus_por_passo = [360 / (steps_per_rev_motor * 64 * r) for r in reducoes]

def direta(t1, t2, t3, t4):
    """Calcula a posição da garra (ponto final do braço) dada as posições angulares (radianos)."""
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
    """Converte um ângulo (radianos) para o número inteiro de passos do motor."""
    return int(round(np.degrees(delta_rad) / graus_por_passo[motor_idx]))

def delta_theta(theta_dest, theta_atual):
    """Calcula a menor diferença angular, tratando a passagem de 360/0 graus."""
    d = theta_dest - theta_atual
    while d > np.pi: d -= 2*np.pi
    while d < -np.pi: d += 2*np.pi
    return d

def erro_angulos(thetas, x_desejado, y_desejado, z_desejado):
    """Função de erro para o fsolve (Cinemática Inversa Absoluta)."""
    t1, t2, t3, t4 = thetas
    xs, ys, zs = direta(t1, t2, t3, t4)
    x_atual, y_atual, z_atual = xs[-1], ys[-1], zs[-1]
    return [
        x_atual - x_desejado,
        y_atual - y_desejado,
        z_atual - z_desejado,
        t2 + t3 + t4
    ]

def inversa_fsolve(x, y, z, chute_inicial):
    """Encontra os ângulos de junta para uma coordenada (x, y, z) usando fsolve."""
    t1_chute = np.arctan2(y, x) if x != 0 or y != 0 else chute_inicial[0]
    chute_inicial[0] = t1_chute
    sol = fsolve(erro_angulos, chute_inicial, args=(x, y, z))
    return sol


def calcular_jacobiano(t1, t2, t3, t4, delta=1e-6):
    """Calcula a matriz Jacobiana da cinemática direta."""
    f0 = np.array(direta(t1, t2, t3, t4))
    pos0 = np.array([f0[0,-1], f0[1,-1], f0[2,-1]])
    J = np.zeros((3,4))
    thetas = [t1, t2, t3, t4]
    
    for i in range(4):
        dtheta = np.zeros(4)
        dtheta[i] = delta
        
        args = [thetas[j] + dtheta[j] for j in range(4)]
        f1 = np.array(direta(*args))
        pos1 = np.array([f1[0,-1], f1[1,-1], f1[2,-1]])
        
        J[:,i] = (pos1 - pos0)/delta
    return J