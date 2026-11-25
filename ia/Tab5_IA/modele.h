// ATTENTION : modele.h D EXEMPLE, entraine sur des mesures SIMULEES. A remplacer apres vos vraies mesures (python analyse_reseau.py analyse).
// modele.h – réseau de neurones généré par analyse_reseau.py le 25/11/2026 16:04
// Entrées : det1, rssi1, det2, rssi2
// Couches : [4, 4, 3] | activation : tanh | précision estimée : 89.2 %
// Niveaux : 0 = LOIN (>= 2.5 m), 1 = PROCHE, 2 = TRES PROCHE (< 1.0 m)
#pragma once

const int N_ENTREES = 4;
const int N_COUCHES = 2;   // nombre de calculs (couches cachées + sortie)
const int TAILLES[3] = {4, 4, 3};
const bool ACTIVATION_RELU = false;   // sinon tanh
const bool UTILISE_HEURE = false;
const bool UTILISE_TEMPERATURE = false;
const float SEUIL_TRES_PROCHE_M = 1.00f, SEUIL_PROCHE_M = 2.50f;
// normalisation : x_norm = (x - MOYENNE) / ECART
const float MOYENNE[4] = {0.9737037f, -73.79096f, 0.862963f, -69.65741f};
const float ECART[4] = {0.160015f, 9.538758f, 0.3438864f, 15.27167f};
// couche 1 : 4 -> 4  (poids rangés [entrée][sortie])
const float W0[16] = {1.757953f, -0.3542543f, 0.6648762f, -0.5746149f, 0.5973171f, 1.423455f, -0.9307888f, 0.915244f, 3.012799f, 0.2228836f, 0.6388673f, -0.09335792f, -2.266322f, 1.261452f, 0.411434f, -2.96624f};
const float B0[4] = {0.8920364f, 0.2606541f, -1.190756f, 0.5290125f};
// couche 2 : 4 -> 3  (poids rangés [entrée][sortie])
const float W1[12] = {2.887752f, 1.442645f, -2.980822f, 0.04670374f, -1.076738f, 1.183333f, -1.355196f, -0.2833389f, 1.476412f, 1.933033f, -2.458733f, -1.435034f};
const float B1[3] = {0.3015163f, 0.6616038f, -1.9566f};
const float* const POIDS[2] = {W0, W1};
const float* const BIAIS[2] = {B0, B1};
