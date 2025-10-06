#let dataflow = [
  notes de https://schedulingseminar.com/presentations/SchedulingSeminar_AlixMunierKordon.pdf
notations : 
- N : vecteur de répétition minimal
- M = lcm($N_i$)
- $Z_i$ = $M/N_i$ pour une tâche $t_i$.


- un graphe est consistant si un vecteur de répétition existe
- le poids d'un cycle W(c) est le produit des u(a)/v(a) : u(a) taux de prod, v(a) taux de conso, a tous le buffers/arcs du cycle
- Si W(c) < 1 (nombre de tokens décroissant) un deadlock arrivera, si W(c) > 1 (nb de tokens croissant) les buffers oxploseront.
- Graphe unitaire : W(c) = 1 pour tout les cycles. Consistant <==> unitaire.
- calculer la consistance d'un graphe prendl un temps polynomial avec l'algo de Belleman-Ford.
- acteur non re-entrant : 2 exécutions de l'acteur ne peuvent pas se superposer.
- inital marking : le nombre de tokens initiaux dans un buffer. Un bon nombre est proportionnel au gcd det taux de prod et conso.
- débit d'un acteur sous un schedule s : $l(t) = lim_n n/(s(t,n))$, avec s(t,n) les dates de la n-ième exécution de t. En gros le nombre moyen de fois qu'un acteur s'exécute dans la partie périodique.
- Si G est consistant et fortement connecté, $l(t) * Z_t$ est une constante.
- le débit d'un graphe est $"Th"(G) = l(a)/N_a$ pour n'importe quel acteur a.
- scheduling K-périodique : en gros, s(t, q + K) = s(t, q) + w. Le débit de l'acteur est K/w, le débit du schedule est Z

]