"""
=============================================================
 Contrôleur Webots - Pendule Inversé
 Basé sur les paramètres du rapport (.wbt) :
   G(s) = -413.01 / (s² - 158.01)
 Modes : PID (angle seul) | LQR (angle + position)
=============================================================
"""

from controller import Robot, Keyboard, Emitter, Receiver
from math import atan, pi
import numpy as np

# ──────────────────────────────────────────────
#  PARAMÈTRES PHYSIQUES (Table 1 du rapport)
# ──────────────────────────────────────────────
M        = 0.10     # Masse des roues [kg]
m        = 0.50     # Masse châssis [kg]
l        = 0.08     # Distance axe → centre de masse [m]
g_acc    = 9.81     # Gravité [m/s²]
R        = 0.065    # Rayon roues [m]
I        = 0.00195  # Moment d'inertie châssis [kg.m²]
tau_max  = 10.0     # Couple max moteur [N.m]

# Constantes dérivées de G(s) (calculées dans le rapport)
q        = I*(M + m) + M*m*l**2          # = 0.00149 kg².m²
K_num    = -(m*l) / (R*q)               # = -413.01
omega0_sq = ((M+m)*m*g_acc*l) / q       # = 158.01  (pôle instable = +12.57 rad/s)

# ──────────────────────────────────────────────
#  GAINS PID  (équivalents MATLAB)
#  T(t) = Kp*e + Ki*∫e + Kd*ė
#  Négatifs car K_num < 0
# ──────────────────────────────────────────────
PID_Kp = -6500.0  # Augmenté depuis -8000 qui donnait 7s
PID_Ki =  -0.5     # Quasi nul : évite tout windup
PID_Kd = -75000   # Augmenté depuis -5700 pour mieux anticiper

# ──────────────────────────────────────────────
#  GAINS LQR  (conservés depuis ton code original)
# ──────────────────────────────────────────────
K_lqr = [14.0, -1.2, 165.0, 615.0]  # [phi, dphi, theta, dtheta] — valeurs optimisées

# ──────────────────────────────────────────────
#  INITIALISATION ROBOT
# ──────────────────────────────────────────────
robot    = Robot()
TIMESTEP = int(robot.getBasicTimeStep())
timestep = TIMESTEP

maxSpeed = 15.0   # Limite vitesse angulaire roues [rad/s]

# Moteurs
motorNames = ['leftMotor', 'rightMotor']
motor = []
for name in motorNames:
    m_dev = robot.getDevice(name)
    m_dev.setPosition(float('inf'))
    m_dev.setVelocity(0)
    motor.append(m_dev)

# Capteurs
accMeter = robot.getDevice('accel');  accMeter.enable(TIMESTEP)
gyro     = robot.getDevice('gyro');   gyro.enable(TIMESTEP)
lEnc     = robot.getDevice('leftEncoder');  lEnc.enable(TIMESTEP)
rEnc     = robot.getDevice('rightEncoder'); rEnc.enable(TIMESTEP)

# Communication
emitter  = robot.getDevice('emitter');  emitter.setChannel(2)
reciever = robot.getDevice('receiver'); reciever.enable(timestep); reciever.setChannel(1)

# Clavier
keyb = Keyboard(); keyb.enable(TIMESTEP)

robot.step(timestep)  # premier pas pour initialiser les capteurs

# ──────────────────────────────────────────────
#  VARIABLES D'ÉTAT
# ──────────────────────────────────────────────
gAng    = 0.0
angle   = 0.0
oT      = robot.getTime()
oldT    = oT
fitness = 0.0
stop    = False

lDisRef = lEnc.getValue()
rDisRef = rEnc.getValue()
oldxD   = 0.0
xD      = 0.0

# Choix du contrôleur : True = PID | False = LQR
use_pid = True   # ← change ici pour basculer de mode

# ──────────────────────────────────────────────
#  ÉTAT PID  (intégrateur + mémoire erreur)
# ──────────────────────────────────────────────
pid_integral   = 0.0
pid_prev_error = 0.0
pid_windup_lim = tau_max   # Anti-windup : limite l'intégrale

# ──────────────────────────────────────────────
#  FONCTIONS
# ──────────────────────────────────────────────

def setSpeed(left, right):
    motor[0].setVelocity(left)
    motor[1].setVelocity(right)

def clamp(val, limit):
    """Sature val dans [-limit, +limit]"""
    return max(-limit, min(limit, val))

def torque_to_speed(torque):
    """
    Convertit le couple T [N.m] en vitesse angulaire roue [rad/s].
    Relation simplifiée : ω = T / (m_total * R²)  (approx.)
    On normalise juste sur maxSpeed pour rester dans les limites Webots.
    """
    return clamp(torque / tau_max * maxSpeed, maxSpeed)

def pid_control(angle_rad, angle_dot, dt):
    """
    Correcteur PID basé sur G(s) = -413.01 / (s² - 158.01)
    Entrée  : angle [rad], dérivée angle [rad/s], pas de temps [s]
    Sortie  : commande vitesse [rad/s] envoyée aux moteurs
    """
    global pid_integral, pid_prev_error

    e     = 0.0 - angle_rad          # erreur = référence (0°) - mesure
    pid_integral += e * dt

    # Anti-windup : on gèle l'intégrale si la commande sature
    integral_contribution = PID_Ki * pid_integral
    if abs(integral_contribution) > pid_windup_lim:
        pid_integral -= e * dt       # annule le dernier incrément

    de_dt = (e - pid_prev_error) / dt if dt > 1e-6 else 0.0
    pid_prev_error = e

    # Couple calculé [N.m]
    torque = PID_Kp * e + PID_Ki * pid_integral + PID_Kd * de_dt

    # Saturation couple physique (τ_max = ±10 N.m)
    torque = clamp(torque, tau_max)

    # Conversion couple → vitesse roue
    speed = torque_to_speed(torque)
    return speed

def lqr_control(phi, dphi, theta_pos, theta_dot):
    """
    Contrôleur LQR : u = -K * [phi, dphi, theta, dtheta]
    Gains K conservés depuis le code original.
    """
    state = [phi, dphi, theta_pos, theta_dot]
    u     = sum(K_lqr[i] * state[i] for i in range(4))
    return clamp(u, maxSpeed)

def reset_param():
    global gAng, angle, lDisRef, rDisRef, oldxD, oT, oldT, xD
    global pid_integral, pid_prev_error, fitness, stop

    gAng    = 0.0
    angle   = 0.0
    fitness = 0.0
    stop    = False

    pid_integral   = 0.0
    pid_prev_error = 0.0

    for i, name in enumerate(motorNames):
        motor[i].setPosition(float('inf'))
        motor[i].setVelocity(0)

    lDisRef = lEnc.getValue()
    rDisRef = rEnc.getValue()
    oldxD   = (lDisRef + rDisRef) * 90 / pi
    oT      = robot.getTime()
    oldT    = oT
    xD      = 0.0

# ──────────────────────────────────────────────
#  BOUCLE PRINCIPALE
# ──────────────────────────────────────────────
while robot.step(timestep) != -1:

    # ---- Communication superviseur ----
    if reciever.getQueueLength() > 0:
        message = reciever.getString()

        if message == "return_fitness":
            emitter.send(str(fitness).encode('utf-8'))
            setSpeed(0, 0)
            reset_param()
            stop = True
            print(f"[Robot] Fitness pour K={K_lqr} → {fitness:.2f}")

        else:
            params = [float(p) for p in message.strip().split(',')]
            K_lqr  = params
            fitness = 0.0
            stop    = False

        reciever.nextPacket()

    # ---- Lecture capteurs ----
    [ax, ay, az] = accMeter.getValues()
    [gx, gy, gz] = gyro.getValues()

    dt   = robot.getTime() - oT
    oT   = robot.getTime()
    if dt <= 0:
        dt = timestep / 1000.0   # fallback sécurité

    # Angle fusionné gyro + accéléromètre (filtre complémentaire)
    gAng += gx * dt * 180 / pi

    aAng  = -atan(gz / gy) * 180 / pi if gy != 0 else 0.0
    aAng  = (90 - aAng) if aAng > 0 else -(90 + aAng)

    angle = 0.99 * gAng + 0.01 * aAng   # angle en degrés
    angle_rad = angle * pi / 180         # → radians pour PID

    # ---- Sélection contrôleur ----
    if stop:
        speed = 0.0

    elif use_pid:
        # ── Mode PID (G(s) du rapport) ──
        speed = pid_control(angle_rad, gx, dt)

    else:
        # ── Mode LQR (original) ──
        lDis = lEnc.getValue() - lDisRef
        rDis = rEnc.getValue() - rDisRef
        xD   = (lDis + rDis) * 90 / pi

        dT_enc = robot.getTime() - oldT
        xD_dot = (xD - oldxD) / dT_enc if dT_enc > 1e-6 else 0.0

        speed = lqr_control(xD, xD_dot, angle, gx)

        oldxD = xD
        oldT  = robot.getTime()

    # ---- Calcul fitness ----
    abs_angle = abs(angle)
    if abs_angle < 5:
        fitness += 0.5
    elif abs_angle < 50:
        fitness += 1.5
    if abs_angle > 50:
        fitness += 100.0

    # ---- Envoi commande moteurs ----
    setSpeed(speed, speed)

print("Simulation terminée.")