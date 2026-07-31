

from controller import Robot, Keyboard, Emitter, Receiver
from math import atan, pi, sqrt, exp
import numpy as np


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



Ms   = 2.0
wb   = 2.0 * omega0     # ≈ 25.14 rad/s
eps  = 0.001
Mu   = 1.0 / tau_max    # = 0.1
gamma_hinf = 1.8        # niveau H∞



abs_Knum = abs(K_num)   # 413.01

Kp_h  =  (2.0 * gamma_hinf * omega0)        / abs_Knum          # ≈  0.1096
Kd_h  =  (gamma_hinf * omega0_sq * Ms)      / (abs_Knum * wb)   # ≈  0.0545
Ki_h  =  (wb * eps * gamma_hinf)            / abs_Knum          # ≈  0.000219
tau_f =  1.0 / (10.0 * omega0)                                  # ≈  0.00796 s


Wc = 8000.0 * 0.0175 / Kp_h   

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


hinf_integral   = 0.0      # ∫e dt
hinf_deriv_filt = 0.0      # état filtre dérivée : ẋ_f = -x_f/τ + e
hinf_prev_error = 0.0

# Anti-windup H∞ : limite l'intégrale
windup_limit = tau_max / (Ki_h * Wc + 1e-9)
windup_limit = min(windup_limit, 500.0)   # borne de sécurité



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

    e = 0.0 - angle_rad   

   
    alpha         = dt / (tau_f + dt)
    x_f_new       = hinf_deriv_filt * (1.0 - alpha) + e * alpha
    e_filt_dot    = (x_f_new - hinf_deriv_filt) / dt if dt > 1e-6 else 0.0
    hinf_deriv_filt = x_f_new

    
    u_no_int = Wc * (Kp_h * e + Kd_h * e_filt_dot)

  
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


while robot.step(timestep) != -1:

  
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


    [ax, ay, az] = accMeter.getValues()
    [gx, gy, gz] = gyro.getValues()

    dt  = robot.getTime() - oT
    oT  = robot.getTime()
    if dt <= 0:
        dt = dt_s

    
    gAng += gx * dt * 180.0 / pi
    aAng  = -atan(gz / gy) * 180.0 / pi if gy != 0 else 0.0
    aAng  = (90.0 - aAng) if aAng > 0 else -(90.0 + aAng)
    angle = 0.99 * gAng + 0.01 * aAng

    angle_rad = angle * pi / 180.0


    if stop:
        speed  = 0.0
        torque = 0.0
    else:
        speed, torque = hinf_control(angle_rad, gx, dt)

   
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
