import numpy as np
import random

class personne:
    def __init__(self, id, age, couleur, en_couple, generation=0):
        self.id = id
        self.age = age
        self.couleur = couleur
        self.en_couple = en_couple
        self.generation = generation

    def __repr__(self):
        return f"Personne(age={self.age}, couleur='{self.couleur}', en_couple={self.en_couple}, generation={self.generation})"
    
    def accoupler(self, autre):
        if not self.en_couple and not autre.en_couple and self.id != autre.id:
            self.en_couple = autre.id
            autre.en_couple = self.id
    
    def reproduire(self, autre, next_id):
        if self.en_couple != None:
            couleur_enfant = 0
            if self.couleur == autre.couleur:
                couleur_enfant = self.couleur
            elif (self.couleur == 0 and autre.couleur == 1) or (self.couleur == 1 and autre.couleur == 0):
                couleur_enfant = 10
            elif autre.couleur == 10:
                couleur_enfant = self.couleur
            else:
                couleur_enfant = autre.couleur
                
        return personne(id=next_id, age=0, couleur=couleur_enfant, en_couple=None, generation=max(self.generation, autre.generation) + 1)
       

def main():
    gen = 10
    # Création d'une instance de la classe personne
    population = np.array([personne(id=i, age=np.random.randint(20, 40), couleur=random.choices([0, 1], weights=[55, 45], k=1)[0], en_couple=None, generation=0) for i in range(50)])
    for g in range(gen):
        print(f"--- Génération {g} ---")
        morts = []
        for p in population:
            if p.age > 82:
                morts.append(p)
            p.age += 20

        population = np.array([p for p in population if p not in morts])
        personnes_par_id = {p.id: p for p in population}

        n_tot = len(population)
        couleurs = {0: 0, 1: 0, 10: 0}
        for p in population:
            couleurs[p.couleur] += 1
        n_0 = couleurs[0]
        n_1 = couleurs[1]
        n_10 = couleurs[10]
        print(f"Population: {n_tot} | Couleur 0: {n_0} proportion: {n_0/n_tot:.2f} | Couleur 1: {n_1} proportion: {n_1/n_tot:.2f} | Couleur 10: {n_10} proportion: {n_10/n_tot:.2f}")
        
        # Accouplement aléatoire
        celibataires = [p for p in population if p.en_couple is None]

        random.shuffle(celibataires)

        for i in range(0, len(celibataires)-1, 2):
            a = celibataires[i]
            b = celibataires[i+1]

            if a.couleur == b.couleur == 1 and random.random() < 0.82:
                a.accoupler(b)

            elif (a.couleur == 1 and b.couleur == 0) or (a.couleur == 0 and b.couleur == 1) and random.random() < 0.25:
                a.accoupler(b)

            elif random.random() < 0.70:
                a.accoupler(b)
        
        # Reproduction
        next_id = len(population)
        enfants = []
        for p in population:
            if p.en_couple is not None and random.random() < 0.9:
                partenaire = personnes_par_id.get(p.en_couple)
                if partenaire:
                    enfant = p.reproduire(partenaire, next_id)
                    enfants.append(enfant)
                    next_id += 1
        
        # Mise à jour de la population pour la prochaine génération
        population = np.concatenate((population, np.array(enfants)))
main()