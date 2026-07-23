"""
=============================================================
 population.py — Algorithme Génétique
 Bug corrigé : mutate() index décalé (i==0 mutait c[1])
=============================================================
"""

import pandas as pd
import random

ELITE_PART           = 0.4
MUTATION_PROBABILITY = 0.45
MUTATION_DEVIATION   = 0.4


def population_create(p_size, geno_size, bounds):
    """Crée une population aléatoire dans les bornes données."""
    return [random_param(geno_size, bounds) for _ in range(p_size)]


def random_param(g, b):
    """Génotype aléatoire uniforme dans les bornes."""
    return [random.uniform(b[i][0], b[i][1]) for i in range(g)]


def population_reproduce(p, fitness):
    """Sélection élitiste + croisement + mutation."""
    size_p = len(p)

    df = pd.DataFrame({"Param": p, "Fitness": fitness})
    df = df.sort_values('Fitness').reset_index(drop=True)
    sorted_p = df['Param'].tolist()

    elite_count = round(ELITE_PART * size_p)
    new_p = sorted_p[:elite_count]  # Élites conservés directement

    for _ in range(size_p - elite_count):
        mom   = p[random.randint(0, size_p - 1)]
        dad   = p[random.randint(0, size_p - 1)]
        child = crossover(mom, dad)
        child = mutate(child)
        new_p.append(child)

    return new_p


def population_get_fittest(p, f):
    """Retourne le meilleur génotype et sa fitness."""
    df = pd.DataFrame({"Param": p, "Fitness": f})
    df = df.sort_values('Fitness').reset_index(drop=True)
    return df.iloc[0]['Param'], df.iloc[0]['Fitness']


def population_get_average_fitness(f):
    return sum(f) / len(f)


def crossover(p1, p2):
    """Croisement uniforme gène par gène."""
    return [
        p2[i] if random.randint(0, 8) > 4 else p1[i]
        for i in range(len(p1))
    ]


def mutate(c):
    """
    Mutation gaussienne par gène.
    ── CORRECTION : chaque gène i mute c[i] (plus le décalage i==0 → c[1])
    ── Amplitudes adaptées aux plages de chaque gain LQR :
         i=0 : K_phi      [3,17]      → σ=2
         i=1 : K_dphi     [-3,2]      → σ=1
         i=2 : K_theta    [100,180]   → σ=5
         i=3 : K_dtheta   [560,640]   → σ=10
    """
    sigmas = [2.0, 1.0, 5.0, 10.0]

    for i in range(len(c)):
        if random.random() < MUTATION_PROBABILITY:
            c[i] += random.gauss(0, sigmas[i])   # ← corrigé : c[i] et non c[i+1]

    return c