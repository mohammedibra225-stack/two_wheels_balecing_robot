"""
=============================================================
 Superviseur - Optimisation Génétique des Gains LQR
 Bugs corrigés :
   - restore_robot_position() variables globales/locales
   - getData() → getString()
   - run_seconds(reset) appelé une seule fois
   - mutate() index décalé corrigé
=============================================================
"""

from controller import Supervisor, Keyboard
from population import *

# ──────────────────────────────────────────────
#  INITIALISATION SUPERVISEUR
# ──────────────────────────────────────────────
superv   = Supervisor()
timestep = int(superv.getBasicTimeStep())
superv.step(timestep)

sbr  = superv.getFromDef("SBR")
load = superv.getFromDef("LOAD")

emitter  = superv.getDevice("emitter")
emitter.setChannel(1)

reciever = superv.getDevice("receiver")
reciever.enable(timestep)
reciever.setChannel(2)

# ──────────────────────────────────────────────
#  PARAMÈTRES ALGO GÉNÉTIQUE
# ──────────────────────────────────────────────
POPULATION_SIZE  = 10
GENOTYPE_SIZE    = 4
NUM_GENERATIONS  = 15

# Bornes pour chaque gain LQR : [K0, K1, K2, K3]
bounds = [(3, 17), (-3, 2), (100, 180), (560, 640)]

# ──────────────────────────────────────────────
#  RÉFÉRENCES POSITION (initialisées dans main)
# ──────────────────────────────────────────────
init_translation      = None
init_rotation         = None
load_init_translation = None
load_init_rotation    = None

# Fields (objets Webots pour setSFVec3f)
sbr_translation_field  = None
sbr_rotation_field     = None
load_translation_field = None
load_rotation_field    = None

# ──────────────────────────────────────────────
#  FONCTIONS UTILITAIRES
# ──────────────────────────────────────────────

def restore_robot_position():
    """Remet le robot et la charge à leur position initiale."""
    sbr_translation_field.setSFVec3f(list(init_translation))
    sbr_rotation_field.setSFRotation(list(init_rotation))
    load_translation_field.setSFVec3f(list(load_init_translation))
    load_rotation_field.setSFRotation(list(load_init_rotation))


def run_seconds(t, reset_position=False):
    """
    Fait tourner la simulation pendant t secondes.
    Si reset_position=True, remet le robot en place UNE SEULE FOIS
    au début (pas à chaque step).
    """
    if reset_position:
        restore_robot_position()
        sbr.resetPhysics()
        load.resetPhysics()

    start = superv.getTime()
    while superv.step(timestep) != -1:
        if superv.getTime() - start > t:
            break


def send_genotype(genotype):
    """Envoie les gains LQR au contrôleur robot."""
    msg = ','.join([str(g) for g in genotype])
    emitter.send(msg.encode('utf-8'))


def getPerformanceData():
    """
    Demande le fitness au robot, attend la réponse,
    puis calcule le coût total (angle + déplacement robot + charge).
    """
    emitter.send("return_fitness".encode('utf-8'))

    while superv.step(timestep) != -1:
        if reciever.getQueueLength() > 0:
            # ── CORRECTION : getString() et non getData() ──
            message      = reciever.getString()
            angle_fitness = float(message)
            reciever.nextPacket()

            # Coût déplacement de la CHARGE
            load_translation = load.getField("translation").getSFVec3f()
            load_rotation    = load.getField("rotation").getSFRotation()
            load_t_cost = sum((a - b)**2 for a, b in zip(load_translation, load_init_translation))
            load_r_cost = sum((a - b)**2 for a, b in zip(load_rotation,    load_init_rotation))

            # Coût déplacement du ROBOT
            sbr_translation = sbr.getField("translation").getSFVec3f()
            sbr_rotation    = sbr.getField("rotation").getSFRotation()
            sbr_t_cost = sum((a - b)**2 for a, b in zip(sbr_translation, init_translation))
            sbr_r_cost = sum((a - b)**2 for a, b in zip(sbr_rotation,    init_rotation))

            total_fitness = (
                angle_fitness
                + (load_r_cost + load_t_cost) * 100 * 30
                + (sbr_r_cost  + sbr_t_cost)  * 30
            )

            print(f"  ├─ Angle fitness   : {angle_fitness:.2f}")
            print(f"  ├─ Load fitness    : {(load_r_cost + load_t_cost):.4f}")
            print(f"  ├─ Robot fitness   : {(sbr_r_cost  + sbr_t_cost):.4f}")
            print(f"  └─ Total fitness   : {total_fitness:.2f}")

            return total_fitness

    # Si on sort sans réponse (timeout)
    print("  [WARN] Pas de réponse fitness reçue.")
    return 9999.0


def evaluate_genotype(genotype):
    """
    Envoie un génotype, lance la simulation 90s,
    récupère le fitness, remet le robot en place.
    """
    print(f"  Envoi génotype : {[f'{g:.2f}' for g in genotype]}")
    send_genotype(genotype)

    # Simulation active
    run_seconds(90)

    # Récupération fitness
    fitness = getPerformanceData()

    # Remise en place
    sbr.resetPhysics()
    restore_robot_position()
    run_seconds(3, reset_position=False)   # petite pause stabilisation
    sbr.resetPhysics()
    restore_robot_position()

    return fitness


# ──────────────────────────────────────────────
#  BOUCLE OPTIMISATION
# ──────────────────────────────────────────────

def run_optimization(population):
    print("\n" + "="*50)
    print(" DÉMARRAGE OPTIMISATION GÉNÉTIQUE")
    print(f" Population : {POPULATION_SIZE}  |  Gènes : {GENOTYPE_SIZE}  |  Générations : {NUM_GENERATIONS}")
    print("="*50 + "\n")

    best_overall    = None
    best_overall_val = float('inf')

    for gen in range(NUM_GENERATIONS):
        print(f"\n{'─'*40}")
        print(f" Génération {gen + 1} / {NUM_GENERATIONS}")
        print(f"{'─'*40}")

        population_fitness = []

        for ind, genotype in enumerate(population):
            print(f"\n  Individu {ind + 1}/{POPULATION_SIZE}")
            fitness = abs(evaluate_genotype(genotype))
            population_fitness.append(fitness)

        best_geno, best_val = population_get_fittest(population, population_fitness)
        avg_val             = population_get_average_fitness(population_fitness)

        print(f"\n  ★ Meilleur génotype   : {[f'{g:.3f}' for g in best_geno]}")
        print(f"  ★ Meilleur fitness    : {best_val:.4f}")
        print(f"  ★ Fitness moyen       : {avg_val:.4f}")

        if best_val < best_overall_val:
            best_overall_val = best_val
            best_overall     = best_geno
            print(f"  ✔ Nouveau meilleur global !")

        if gen < NUM_GENERATIONS - 1:
            population = population_reproduce(population, population_fitness)

    print(f"\n{'='*50}")
    print(f" OPTIMISATION TERMINÉE")
    print(f" Meilleur génotype : {[f'{g:.4f}' for g in best_overall]}")
    print(f" Meilleur fitness  : {best_overall_val:.4f}")
    print(f"{'='*50}\n")

    return best_overall


# ──────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────

def main():
    global init_translation, init_rotation
    global load_init_translation, load_init_rotation
    global sbr_translation_field, sbr_rotation_field
    global load_translation_field, load_rotation_field

    keyb = Keyboard()
    keyb.enable(timestep)

    # Sauvegarde des positions initiales
    sbr_translation_field  = sbr.getField("translation")
    sbr_rotation_field     = sbr.getField("rotation")
    load_translation_field = load.getField("translation")
    load_rotation_field    = load.getField("rotation")

    init_translation      = tuple(sbr_translation_field.getSFVec3f())
    init_rotation         = tuple(sbr_rotation_field.getSFRotation())
    load_init_translation = tuple(load_translation_field.getSFVec3f())
    load_init_rotation    = tuple(load_rotation_field.getSFRotation())

    print(f"Position initiale robot  : {init_translation}")
    print(f"Position initiale charge : {load_init_translation}")

    # Création population initiale
    population = population_create(POPULATION_SIZE, GENOTYPE_SIZE, bounds)

    # Lancement optimisation
    fittest = run_optimization(population)

    # Envoi du meilleur génotype trouvé
    print("Envoi du meilleur génotype au robot...")
    send_genotype(fittest)
    restore_robot_position()

    # Boucle attente (Q pour quitter)
    print("Appuie sur Q pour quitter.")
    while superv.step(timestep) != -1:
        key = keyb.getKey()
        if key == ord('Q'):
            break


main()