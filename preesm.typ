#let preesm = [

radio interferometer measurement equation @Smirnov_2011
résumé :
Un point observé dans le ciel $e=mat(e_x ; e_y)$ subit des distorsions (optiques, électroniques...) supposées linéaires. On les représente par des matrices de Jones : $e'=J e$ qui se cumulent : $e'=J_n J_{n-1}...J_1 e = J e$ et se simplifient en une matrice de Jones totale. La mesure de ce signale par une antenne donne une tension complexe (ou vectorielle) $v=mat(v_a ; v_b)=J e$ fonction du signal lumineux venu du ciel. 


Si deux antennes p et q mesurant des vecteurs de tension $v_p, v_q$, on peut corréler leurs mesures pour former un interféromètre et obtenir une matrice de visibilités : $V_"pq" = 2 mat(<v_"pa" v*_"qa">,  <v_"pa" v*_"qb"> ; <v_"pb" v*_"qa", <v_"pb" v*_"qb">) =2<v_p v_q^H>$ où $<>$ dénote une moyenne du signal sur un temps (court). 

Sachant que le signal $e$ traverse 2 chemins différents pour atteindre p et q, on a $V_"pq"=2<J_"pe" J_"qe"^H>=2 <J_p e e^H J_q^H> = 2 J_p <e e^H> J_q^H$ en supposant $J_p, J_q$ constants. Or, la matrice formée par $<e e^H>$  est la matrice de luminosité $B$ (brightness matrix) qu'on cherche ! Donc $V_"pq" = J_p B J_q^H$ donne B en fonction de V, à condition de pouvoir l'inverser.

Les matrices de Jones totales prennent en compte au moins un effet, même dans une observation idéale : la différence de phase relative. Un interféromètre essaye de la minimiser en introduisant des délais pour un des deux signaux. La différence de phase pour une antenne p de coordonnées $u_p=(u_p, v_p, w_p)$ relativement au point de référence $u=0$ observant un signal de direction $sigma$ vaut $kappa_p=2 pi lambda^(-1)(u_"pl" + v_p m + w_p(n-1)$ avec l, m, $n = sqrt(1-l^2-m^2)$ sont les cosinus de direction de $sigma$ (??). \\ 
On peut poser que la matrice de Jones est une matrice scalaire qui vaut $K_p=e^(-i kappa_p) "Id"$, et donc $V_"pq"=K_p B K_p^H=B e^(2 i pi (u_"pq" l + v_"pq" m + w_"pq" (n-1)))
)=X_"pq"$ avec $u_"pq"=u_p - u_q$. On appelle $X_"pq"$ la matrice de cohérence de source.

Dans une vraie observation, on aura également des effets de corruption représentés par $G_p$. La chaîne de Jones devient $J_p=G_p K_p$ où on peut balader $K_p$ à volonté puisque c'est une matrice scalaire. On obtient alors la RIME $V_"pq"=G_p X_"pq" G_q^H$.

Si on observe N points sources ayant chacun leur matrice de Jones, leurs contribution à la matrice de visibilité est linéaire : $V_"pq" = sum_(s) J_"sp" B_s J_"sq"^H$. 

On peut diviser $J_"sp"$ en 3 sources $J_"sp"=G_p E_"sp" K_"sp"$ : G les effets de l'antenne, K le terme de phase, et E les effets de la source. On obtient $V_"pq" = G_p (sum_s E_"sp" K_"sp" B_s K_"sq"^H E_"sq"^H) G_q^H = G_p (sum_(s) E_"sp" X_"spq" E_"sq"^H) G_q^H$. $G_p$ donne les direction-independant effects et $E_"sp"$ les direction-dependant effects.

Dans le cas où la luminosité du ciel est décrite par une distribution continue $B(sigma)$ avec $sigma$ le vecteur de direction, on a une matrice de jones totale dépendant de $sigma$ $J_p(sigma)$. 

On arrive à $V_"pq" = integral_{4\pi} J_p(sigma) B(sigma) J_q^H(sigma) d Omega $ intégrale sur une sphère unité. L'intégrale n'est pas pratique donc on fait une projection sinusoidale dans le plan (l,m) : $V_"pq" = integral_l integral_{m} J_p(l) B(*l*) J_q^H(*l*) "dldm"
/ n $ avec $n= sqrt(1-l²-m²)$ et $*l*=(l,m)$. 

En développant les matrices de Jones en termes direction-dependant et -independant et en terme de phase, et en intégrant le terme de coplanarité $w_"pq"(n-1)$ dans les termes DD, on exprime la RIME comme une TF 2D : $V_"pq" = G_p (integral_l integral_m B_"pq" e^(-2 i pi(u_"pq" l + v_"pq" m) "dl" "dm" ) G_q^H $ où $B_"pq"$ intègre les termes DD et représente donc le ciel apparent vu par la baseline pq : $B_"pq"=E_p B E_q^H$. 

Note que chaque baseline voit donc un ciel différent, à moins que tous les effets DD soient égaux : $E_p(*l*)=E(*l*)$.
"Dans ce cas précis, la RIME peut s'écrire $V_"pq" =G_p X_"pq" G_q^H$ avec $X_"pq"=X(u_"pq", v_"pq")$ et $X(*u*)$ est la TF 2D de $B_"pq"(*l*)$, on note $X=F B_"pq"(*l*)$ quand les DDE sont tous égaux (donc jamais en vrai).



#linebreak()

visibilité en astronomie (ou optique) : contraste entre les franges d'intensité max et min dans un schéma d'interférences : $nu = (I_"max" - I_"min") / (I_"max" + I_"min")$. 
Sachant que $nu = 1$ --> sources parfaitement cohérentes, et que les étoiles sont des objets incohérents (chaque point de sa surface émet indépendamment). 
Une visibilité est calculée par l'autocorrélation entre les tensions mesurées par deux télescopes, c'est donc une matrice : $V equiv E[arrow(v) arrow(v^*)] $ avec $arrow(v)$ les vecteurs des tensions mesurées.
Le calcul de l'intensité des franges d'interférence en tout point fait apparaître que leur intensité lumineuse est liée à la baseline (la distance réparant les deux sources) par une TF. 
La distribution des visibilités (= schéma d'interférences) est proportionnelle à la TF de la distribution de l'intensité lumineuse de l'objet observé, et les baseline en sont les coordonnées de fourier --> plus on a de baseline (donc de télescopes) plus on a une bonne approximation de cette TF. 
La distribution de l'intensité dans le schéma d'interférences est elle liée à la fonction de visibilités. Donc si on connaît les visibilités et les baseline on peut remonter à l'intensité de la source. https://www.eso.org/sci/facilities/paranal/telescopes/vlti/tuto/tutorial_spatial_interferometry.pdf

#linebreak()

Méthode Bluebild : 
Il si trouve que $V = E[arrow(v) arrow(v^*)] = Psi^* B Psi + sigma^2 "Id"$ avec $Psi$ le sampling operator et $B$ la matrice de corrélation de champ électrique discrétisé : $B = E[arrow(E) arrow(E^*)]$. Or, B est une matrice diagonale dont la diagonale vaut $E_j^2$ ! On peut donc retrouver l'intensité lumineuse du ciel $I=|E_j|^2$ en estimant $tilde(B)$ satisfaisant le moindre carré $||Psi^* B Psi - V||^2$ de solution connue $tilde(B) = Psi G_Psi^(-1) V G_Psi^(-1) Psi^*$ avec $G_Psi=Psi^* Psi$, calculable avec le vecteur de baseline pq entre les deux antennes. Mais la matrice de Gram $G$ est souvent mal conditionnée...
Bluebild calcule $tilde(B)$ par sa décompositions en valeurs propres : On arrive à l'équation généralisée de valeurs propres $V alpha_a = lambda_a G_Psi alpha_a$, avec ${(lambda_a, alpha_a)}$ les couples de valeurs/vecteurs propres. On peut alors estimer la distribution d'intensité lumineuse : $tilde(I) = sum_a lambda_a |Psi alpha_a|^2$. Sachant que V et psi dépendent du temps et de la freq, donc tout le calcul est à faire pour tout instant t et bande de freq f. C'est la synthèse d'images standard.
Le calcul de $tilde{I}$ se fait à partir des vecteurs de baseline, et peut s'exprimer comme une TF dont les baseline sont les coordonnées de fourier : $tilde(I)_j=sum_n^(N_a times N_a) V_n^{'} e^(-2 i pi <arrow(b_n), arrow(r_j)>)$ qui est une NUFFT type 3. C'est la synthèse d'image par NUFFT.
Dans le cas où les tensions sont beamformed ensemble selon une matrice de poids W, on prend en compte ces poids dans le sampling operator : $V^W = W^H Psi^*$.
Il est possible d'appliquer des filtres sur les valeurs et vecteurs propres pour plusieurs images, p.e le filtre least-square donne l'image minimisant l'erreur quadratique (appliqué lors du calcul de I présenté plus haut), ou une somme standardisée (sans les $lambda_a$) pour normaliser les flux entre eigenimages (???).

Pour le calcul numérique de la NUFFT, on scale les coordonnées $b_n$ dans l'espace de fourier pour qu'elles tiennent dans $[-pi, pi)³$ avec des facteurs $gamma_i$. L'équation devient $tilde(B)_"pix"=sum_n^N V_n^(') e^(-2i\pi x_"pix"^(') b_n^('))$ avec $x' = gamma_i x$. Ensuite on répartie les coordonnées de baseline sur une grille $"bx"$ avec un kernel périodique tel que $b_x=sum_n^N V_n phi(x_1h_1-b_n^('(1)), x_2h_2-b_n^('(2)), x_3h_3-b_n^('(3)))$ avec $h_i$ l'espacement de la grille et $phi$ le kernel de FINUFFT "exponentiel de semicercle" normalisé et périodisé. On peut enfin appliquer une NUFFT type 2 et appliquer une correction diagonale pour estimer B. \\
Pas de déconvolution pour éliminer la PSF, seulement des actions physiques (modifier la répartition des télescopes...) ou matheuses (modifier le beamforming)

#linebreak()

Méthode DDFacet : 
Utilise du facettage pour utiliser des correcteurs adaptés aux effets de distorsions (pas vraiment le bon terme) dépendant de la direction d'observation. \\
Toute la méthode part de la corrélation entre deux antennes p et q $V_"pq", t\nu$ une matrice 2x2, avec $t$ le temps $\nu$ la fréquence. $V_"pq",t nu$ est aussi égal à une TF des vecteurs d'intensité lumineuse du ciel, dont les coordonnées de fourier sont les vecteurs de baseline $b_{"pq",t}$. On peut écrire cette relation comme une équation linéaire : $V_(b_nu)=A_(b_nu) x_nu$ avec $V_(b_v)$ le "visibility 4-vector" échantillonné par baseline b=(p,q,t) à la fréquence $\nu$. Quand on combine tous les $\nu$ pour former une matrice, on arrive à $v_(nu) = A_(nu) x_(nu)$, et $A_(nu)$ est rarement inversible. Pour contourner ça, on approxime son inverse par son adjointe $A_(nu)^(H)$ (ou une approximation $tilde(A_nu^H)$), ce qui donne la dirty image $y_nu=A_nu^H W_nu v_nu$ avec $W_{nu}$ la matrice de beamforming (je crois ? Sinon "la matrices des poids à la fréquence nu") (supposant une approximation parfaite de $A_nu^H$). Un développement de cette expression selon chaque vecteur de baseline $b_(nu)$ fait apparaître une convolution (une par baseline) : $y_nu="Cx"_nu$ la dirty image est le résultat de la convolution (plus ou moins une vraie convo...) de l'image réelle par la PSF. 

#linebreak()

== semaine 24/02/25
Dans DDFacet, la PSF est construite en prenant en compte les effets dépendants du temps/fréquence/direction. En règle général on ne peut donc pas modéliser l'impact de ces effets par une convolution, mais avec quelques approximations et en estimant (et appliquant) la PSF par facette on peut modéliser ces effets par une convolution locale.

algorithmes d'imagerie :
- DDFacet
  - gridder : Baseline-Dependant Averaging gridder
  - w-terms : gérés par facettage
- Bluebild (implem BIPP)
- WSClean 
  - w-terms : gérés par w-stacking
  - DDE : gérés par a-projection
  - gridding : utilise un sinus cardinal fenêtré (= passe-bas) et du suréchantillonage pour bien placer les samples sur la grille uv ; utilise un kernel Kaiser-Bessel
- Image-Domain Gridding (IDG) :


méthodes de gestion des w-termes (non parallélité du plan des télescopes avec le plan de l'image(=ciel) qui empêche une parfaite approximation de la RIME en une TF 2D) @Offringa_McKinley_Hurley_Walker_Briggs_Wayth_Kaplan_Bell_Feng_Neben_Hughes_et_al @Tol_Veenboer_Offringa_2018: 
- facettage : on fait une TF pour chaque facette, donc l'approximation en TF 2D est davantage valable
- FFT 3D : abandonner l'approximation, au prix du temps de calcul
- w-projection : le kernel de convolution du gridding est multiplié par un terme de phase $e^(2 i pi "wn"')$. On peut réduire la taille du W term avec le w-stacking
- A-projection : extension de w-projection, on multiplie le kernel par $e^(2 i pi "wn"') g_p(l,m) g^*_q(l,m)$ avec $g_p(l,m)$ le "voltage reception pattern" de l'antenne p. (A-term = Antenna term ?)
- w-stacking
- warped snapshots
- w-snapshots
- AW-projection

gridding : son but est de placer les visibilités de coordonnées continues sur une grille de coordonnées discrètes, en plaçant les visibilités sur un plan suréchantillonné puis en réduisant ce plan aux dimensions voulues. Cela équivaut à de la quantification, ce qui cause de l'aliasing et nécessite donc un filtrage basse-bas dans le domaine fréquentiel avant la quantif (équivalent à multiplier l'image de sortie par une fenêtre qui ne garde que ce qu'on veut observer) (pourtant j'en ai pas vu dans l'algo de gridding de DDFacet ???). La version la plus classique de gridding se fait par une convolution, dont le kernel a en gros une tête de gaussienne (mais peut être sinc, exp de semicercle, Kaiser-Bessel...).
Le lien entre la dirty image $y$ et le vecteur contenant les visibilités $v\in R^P$ est $y=F^T S^T v$ avec $F^T$ la matrice de TF inverse et $S^T$ la matrice de gridding.

w-projection @Monnier_Orieux_Gac_Tasse_Raffin_Guibert_2023 : $V(u,v,w) = V(u,v,w=0) * tilde(G)(u,v,w)$ avec $tilde(G)(u,v,w) = "TF"(G(l,m,w))$ et $G(l,m,w)=e^(2 i pi(sqrt(1-l²-m²)-1))$

w-stacking  @Corda_Veenboer_Awan_Romein_Jordans_Kumar_Boonstra_Corporaal_2022: au lieu de faire une convolution dans l’espace $"uv"$, on 1) gride les visibilités sur des w-layers par valeur égale de w, 2) calcule la FFT pour chaque layer, 3) applique le DD phase-shift pour chaque layer, 4) somme les résultats de chaque w-layer puis on scale le résultat.

On va donc transformer cette équation :
$I’(l,m)(w_(max)-w_(min))/sqrt(1-l²-m²) = integral_(w_"min")^(w_"max") e^(2 i pi(sqrt(1-l²-m²)-1)) times integral integral v(u,v,w)e^(2 i pi("ul"+"vm")) "du" "dv" "dw" $
en une somme sur w discrétisé de FFT inverse multipliées par un changement de phase.

Pour w, une discrétisation uniforme est optimale dans ce cas. Pour u, v la discrétisation est classique. Le nombre de w-layers va de 10aines à 100aines selon l’angle d’observation.

NUFFT @Barnett_Magland_Klinteberg_2019 : 2 classes de méthodes : 
- gridding puis FFT suréchantillonnée (je suppose pour passer à une FFT N points avec moins de N échantillons) puis correction dans l'espace de Fourier. Il existe plusieurs variantes de cette méthodes dans le cas 1D. La plus populaire.
- interpolation vers une grille régulière (= pas suréchantillonnée) de N points puis FFT N points

- type 1 nonuniforme -> uniforme :
  - spreading par kernel de conv $psi$
  - FFT
  - extraction des N 1er fréquences et division par la TF de $psi$ évalué pour chaque fréquence (c'est la déconvolution ou roll-off correction)

    Nombreux kernels possibles : truncated Gaussian, B-splines, Kaiser–Bessel, exponential of semicircle et leurs approximations.
- type 2 uniforme -> nonuniforme : mêmes étapes dans l'autre sens. L'inverse du spreading est une interpolation, une convolution par la TF du kernel : $tilde(c)_j=sum\_(l_1=0)^(n_1-1) ... sum_(l_d=0)^(n_d-1) b_l tilde(psi)((l_1 h_1, ..., l_d h_d) - x_j) $, $l = (l_1, ..., l_d)$ et $x_j$ est le point de la grille nonuniforme.
    --> RESSEMBLE VRAIMENT A DU GRIDDING !
- type 3 : 

FINUFFT utilise une exponential of semicircle, plus rapide à évaluer que KB mais quasi aussi précise selon eux.

algorithmes de CLEAN @Monnier :
- Single-scale 
  - Hogbom : extraction du pixel le plus brillant, convolution par la PSF et soustraction du résultat fois un gain à l'image d'entré, ajout du résultat multiplié par un gain à l'image de sortie. Répété n fois.
  - Clark-CLEAN : on approxime la PSF par son centre pour réduire les calculs. La convolution est faite dans le domaine de fourier pour accélérer.
  - Cotton \& Schwarz : pas de convolution par la PSF mais utilisation de la vraie formule sans approximation. Plus précis car w-projection mais très lent.
- Multi-scale CLEAN : plusieurs PSF à choisir selon les circonstances (pas sûr).
- Multi-frequencies : pour traiter une image multi-fréquences 

gridding : basé sur l'interpolation via convolution. On approxime les valeurs continues sur une grille suréchantillonnée de résolution $Delta_u, Delta_v$. La grille de sortie $g$ aura une résolution plus grossière d'un facteur K. La position approximée d'une visibilité est $tilde(b)_i=(p_i Delta_u, q_i Delta_v), p_i, q_i \in "*N*"$. Pour calculer les valeurs de $g$, on fait une convolution : $g(b_k)=sum_{i=1}^{M} C^T (b_k-tilde(b)_i) v_i = sum_(i=1)^M C^T (p_k K Delta_u - p_i Delta_u, q_i K Delta_v - q_i Delta_v)v_i $

]