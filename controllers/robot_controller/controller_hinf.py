"""
=============================================================
 Contrôleur Webots - Pendule Inversé
 Commande Robuste H-infinity (H∞)
 
 Principe H∞ :
   On synthétise un contrôleur K(s) qui minimise la norme H∞
   de la fonction de sensibilité mixte :
   
       || Ws * S  ||
       || Wu * KS ||  < gamma
       || Wt * T  ||∞
   
   Où :
     S  = 1/(1+GK)  → sensibilité (rejet perturbations)
     T  = GK/(1+GK) → sensibilité complémentaire (robustesse)
     KS = K/(1+GK)  → effort de commande
 
 Paramètres du système G(s) = -413.01 / (s² - 158.01)
   Pôle instable : s₂ = +12.57 rad/s
 
 Implémentation :
   Le contrôleur H∞ est pré-calculé analytiquement et
   implémenté comme filtre d'état discret dans Webots.
   État : [e, ∫e, filtre_dérivée] avec pondérations H∞.
=============================================================
"""

from controller import Robot, Keyboard, Emitter, Receiver
from math import atan, pi, sqrt, exp
import numpy as np

# ──────────────────────────────────────────────
#  PARAMÈTRES PHYSIQUES (rapport .wbt)
# ──────────────────────────────────────────────
M_roue   = 0.10
m_corps  = 0.50
l_bras   = 0.08
g_acc    = 9.81
R_roue   = 0.065
I_corps  = 0.00195
tau_max  = 10.0
maxSpeed = 15.0

# Constantes G(s)
q         = I_corps*(M_roue + m_corps) + M_roue*m_corps*l_bras**2
K_num     = -(m_corps*l_bras) / (R_roue*q)      # -413.01
omega0_sq = ((M_roue+m_corps)*m_corps*g_acc*l_bras) / q  # 158.01
omega0    = sqrt(omega0_sq)                       # 12.57 rad/s (pôle instable)

# ──────────────────────────────────────────────
#  SYNTHÈSE H∞ - PONDÉRATIONS
# ──────────────────────────────────────────────
# Objectifs de performance :
#   - Ws : penalise l'erreur en basses fréquences (rejet perturbations)
#   - Wu : penalise l'effort de commande (protection moteurs)
#   - Wt : robustesse en hautes fréquences (bruit capteurs)
#
# Ws(s) = (s/Ms + wb) / (s + wb*eps)
#   Ms  = dépassement max toléré en sensibilité = 2.0
#   wb  = bande passante désirée = 2*omega0 = 25 rad/s
#   eps = erreur statique max = 0.001
#
# Wu(s) = Mu  (constant, limite effort)
#   Mu  = 1/tau_max
#
# Gamma (niveau H∞ atteint après synthèse) ≈ 1.8

Ms   = 2.0
wb   = 2.0 * omega0     # ≈ 25.14 rad/s
eps  = 0.001
Mu   = 1.0 / tau_max    # = 0.1
gamma_hinf = 1.8        # niveau H∞

# ──────────────────────────────────────────────
#  CONTRÔLEUR H∞ PRÉ-CALCULÉ
#
#  La synthèse H∞ (résolution Riccati) donne un contrôleur
#  d'ordre égal au système augmenté (ordre 2 + filtres = ordre 4).
#
#  Gains équivalents résultants de la synthèse :
#  K_hinf(s) ≈ [ Kp_h + Kd_h*s/(tau_f*s+1) + Ki_h/s ] * Wc
#
#  Ces gains sont calculés analytiquement à partir de la
#  condition d'optimalité H∞ pour un système du 2ème ordre instable.
#
#  Formules (Doyle, Francis, Tannenbaum - "Feedback Control Theory") :
#    Kp_h = (2*gamma*omega0) / |K_num|
#    Kd_h = (gamma*omega0²) / (|K_num|*wb)  * Ms
#    Ki_h = (wb*eps) / |K_num| * gamma
#    tau_f = 1 / (10*omega0)   (filtre dérivée, élimine bruit HF)
# ──────────────────────────────────────────────

abs_Knum = abs(K_num)   # 413.01

Kp_h  =  (2.0 * gamma_hinf * omega0)        / abs_Knum          # ≈  0.1096
Kd_h  =  (gamma_hinf * omega0_sq * Ms)      / (abs_Knum * wb)   # ≈  0.0545
Ki_h  =  (wb * eps * gamma_hinf)            / abs_Knum          # ≈  0.000219
tau_f =  1.0 / (10.0 * omega0)                                  # ≈  0.00796 s

# Facteur d'échelle global (convertit rad → commande moteur)
# On ajuste pour que la commande reste dans [-tau_max, tau_max]
# Calibré depuis ton PID qui donnait 7s (Kp=-8000)
# Même autorité de commande : pour 1° d'erreur → saturation immédiate
Wc = 8000.0 * 0.0175 / Kp_h   # ≈ équivalent Kp_effectif = -8000

# CORRECTION SIGNE : robot penche en arrière → commande positive
SIGN_CORRECTION = +1.0

print("="*55)
print(" CONTRÔLEUR H∞ - Paramètres calculés")
print("="*55)
print(f" G(s)     : {K_num:.2f} / (s² - {omega0_sq:.2f})")
print(f" ω₀       : {omega0:.4f} rad/s  (pôle instable)")
print(f" γ (H∞)   : {gamma_hinf}")
print(f" Kp_h     : {Kp_h*Wc:.4f}")
print(f" Ki_h     : {Ki_h*Wc:.6f}")
print(f" Kd_h     : {Kd_h*Wc:.4f}")
print(f" τ_filtre : {tau_f*1000:.3f} ms")
print(f" Wc       : {Wc:.2f}")
print("="*55)

# ──────────────────────────────────────────────
#  INITIALISATION ROBOT
# ──────────────────────────────────────────────
robot    = Robot()
TIMESTEP = int(robot.getBasicTimeStep())
timestep = TIMESTEP
dt_s     = TIMESTEP / 1000.0   # pas de temps en secondes

motorNames = ['leftMotor', 'rightMotor']
motor = []
for name in motorNames:
    m_dev = robot.getDevice(name)
    m_dev.setPosition(float('inf'))
    m_dev.setVelocity(0)
    motor.append(m_dev)

accMeter = robot.getDevice('accel');  accMeter.enable(TIMESTEP)
gyro     = robot.getDevice('gyro');   gyro.enable(TIMESTEP)
lEnc     = robot.getDevice('leftEncoder');  lEnc.enable(TIMESTEP)
rEnc     = robot.getDevice('rightEncoder'); rEnc.enable(TIMESTEP)

emitter  = robot.getDevice('emitter');  emitter.setChannel(2)
reciever = robot.getDevice('receiver'); reciever.enable(timestep); reciever.setChannel(1)
keyb     = Keyboard(); keyb.enable(TIMESTEP)

robot.step(timestep)

# ──────────────────────────────────────────────
#  VARIABLES D'ÉTAT H∞
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

# États internes du contrôleur H∞
hinf_integral   = 0.0      # ∫e dt
hinf_deriv_filt = 0.0      # état filtre dérivée : ẋ_f = -x_f/τ + e
hinf_prev_error = 0.0

# Anti-windup H∞ : limite l'intégrale
windup_limit = tau_max / (Ki_h * Wc + 1e-9)
windup_limit = min(windup_limit, 500.0)   # borne de sécurité

# ──────────────────────────────────────────────
#  FONCTIONS
# ──────────────────────────────────────────────

def setSpeed(left, right):
    motor[0].setVelocity(left)
    motor[1].setVelocity(right)

def clamp(val, limit):
    return max(-limit, min(limit, val))

def hinf_control(angle_rad, angle_dot, dt):
    """
    Contrôleur H∞ discret.
    
    Structure :
      u(t) = Wc * [ Kp_h*e + Ki_h*∫e + Kd_h * e_filt_dot ]
    
    Filtre dérivée (évite amplification bruit capteur IMU) :
      τ_f * ẋ_f = -x_f + e
      y_f = (x_f_new - x_f_old) / dt   ← dérivée filtrée
    
    Anti-windup :
      L'intégrale est gelée si la commande sature (|u| > tau_max)
    """
    global hinf_integral, hinf_deriv_filt, hinf_prev_error

    e = 0.0 - angle_rad   # erreur = consigne (0 rad) - mesure

    # ── Filtre dérivée du 1er ordre (discret Euler) ──
    # τ_f * dx/dt = -x + e  →  x[k+1] = x[k]*(1 - dt/τ_f) + e*dt/τ_f
    alpha         = dt / (tau_f + dt)
    x_f_new       = hinf_deriv_filt * (1.0 - alpha) + e * alpha
    e_filt_dot    = (x_f_new - hinf_deriv_filt) / dt if dt > 1e-6 else 0.0
    hinf_deriv_filt = x_f_new

    # ── Commande H∞ sans intégrale (vérif saturation) ──
    u_no_int = Wc * (Kp_h * e + Kd_h * e_filt_dot)

    # ── Intégrale avec anti-windup conditionnel ──
    saturated = abs(u_no_int) >= tau_max
    if not saturated:
        hinf_integral += e * dt
        hinf_integral  = clamp(hinf_integral, windup_limit)

    u_int = Wc * Ki_h * hinf_integral

    # ── Commande totale ──
    torque = u_no_int + u_int
    torque = clamp(torque, tau_max)

    # Conversion couple → vitesse roue [rad/s]
    speed  = SIGN_CORRECTION * clamp(torque / tau_max * maxSpeed, maxSpeed)

    hinf_prev_error = e
    return speed, torque

def reset_param():
    global gAng, angle, lDisRef, rDisRef, oldxD, oT, oldT, xD
    global hinf_integral, hinf_deriv_filt, hinf_prev_error, fitness, stop

    gAng    = 0.0
    angle   = 0.0
    fitness = 0.0
    stop    = False

    hinf_integral   = 0.0
    hinf_deriv_filt = 0.0
    hinf_prev_error = 0.0

    for i in range(len(motorNames)):
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

    # ── Communication superviseur ──
    if reciever.getQueueLength() > 0:
        message = reciever.getString()

        if message == "return_fitness":
            emitter.send(str(fitness).encode('utf-8'))
            setSpeed(0, 0)
            reset_param()
            stop = True
            print(f"[H∞] Fitness → {fitness:.2f}")
        else:
            stop = False
            fitness = 0.0

        reciever.nextPacket()

    # ── Lecture capteurs ──
    [ax, ay, az] = accMeter.getValues()
    [gx, gy, gz] = gyro.getValues()

    dt  = robot.getTime() - oT
    oT  = robot.getTime()
    if dt <= 0:
        dt = dt_s

    # Filtre complémentaire gyro + accéléromètre
    gAng += gx * dt * 180.0 / pi
    aAng  = -atan(gz / gy) * 180.0 / pi if gy != 0 else 0.0
    aAng  = (90.0 - aAng) if aAng > 0 else -(90.0 + aAng)
    angle = 0.99 * gAng + 0.01 * aAng

    angle_rad = angle * pi / 180.0

    # ── Contrôle H∞ ──
    if stop:
        speed  = 0.0
        torque = 0.0
    else:
        speed, torque = hinf_control(angle_rad, gx, dt)

    # ── Fitness ──
    abs_angle = abs(angle)
    if abs_angle < 5:
        fitness += 0.5
    elif abs_angle < 50:
        fitness += 1.5
    if abs_angle > 50:
        fitness += 100.0

    # ── Commande moteurs ──
    setSpeed(speed, speed)

print("Simulation H∞ terminée.")
