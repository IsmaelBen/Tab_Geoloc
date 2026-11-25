/*
 * TAB-Géoloc – Interface « GPS rétro » pour le test Wi-Fi entre 2 M5Stack Tab5
 * ------------------------------------------------------------------------------
 * Même programme pour les deux Tab5 : changer seulement ROLE avant de téléverser.
 *   ROLE 1 = Tab5 FIXE   (joue le rôle de la balise / cible)
 *   ROLE 2 = Tab5 MOBILE (porté par l'utilisateur : distance estimée, tendance, signal)
 *
 * Carte « M5Tab5 » (paquet M5Stack >= 3.2.2), bibliothèques M5Unified >= 0.2.23 et M5GFX >= 0.2.30.
 * Toucher l'écran du Tab5 mobile = remise à zéro des compteurs (bip).
 */

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// ======================= À CHANGER AVANT DE TÉLÉVERSER =======================
#define ROLE 2            // 1 = Tab5 fixe, 2 = Tab5 mobile
#define MODE_PARTAGE 0    // 0 = le Tab5 fixe crée le Wi-Fi ; 1 = partage de connexion d'un téléphone
const char* PARTAGE_SSID = "NomDuPartage";
const char* PARTAGE_MDP  = "MotDePasse";

// Calibration de la distance : mettre le RSSI que vous avez mesuré à 1 m pendant le test
const float RSSI_1M  = -35.0;   // dBm à 1 m (mesuré)
const float N_PROPAG = 2.5;     // 2 = espace libre, 3 = intérieur encombré
// =============================================================================

const char* AP_SSID = "TAB-GEOLOC-TEST";
const char* AP_MDP  = "tabgeoloc2026";
const uint16_t PORT = 4210;
const uint32_t PERIODE_PING_MS = 200;
const int SDIO_CLK = 12, SDIO_CMD = 13, SDIO_D0 = 11, SDIO_D1 = 10, SDIO_D2 = 9, SDIO_D3 = 8, SDIO_RST = 15;

WiFiUDP udp;
M5Canvas ecran(&M5.Display);     // on dessine tout dans une image, puis on l'affiche d'un coup (pas de scintillement)
int W, H;
char buf[64];

// Palette « écran LCD » : fond vert-gris, encre sombre, segments éteints très pâles
uint16_t C_FOND, C_ENCRE, C_FANTOME, C_GRIS;

// --- mesures (Tab5 mobile)
uint32_t numero = 0, envoyes = 0, recus = 0, somme_rtt = 0;
uint32_t dernier_ping = 0, dernier_dessin = 0, dernier_csv = 0, dernier_pong = 0;
float rssi_lisse = NAN;
const int N_HIST = 15;            // 3 s d'historique (1 valeur / 200 ms) pour la tendance
float historique[N_HIST];
int idx_hist = 0, nb_hist = 0;
const int N_MED = 9;              // médiane des 9 dernières mesures (≈ 2 s) : supprime les valeurs aberrantes
float brut[N_MED];
int idx_brut = 0, nb_brut = 0;

// Plages de distance affichées (en mètres) : plus honnête qu'un chiffre précis au mètre près
const float BORNES[3] = {1.0f, 3.0f, 6.0f};
const char* NOM_ZONE[4]   = {"TRES PROCHE", "PROCHE", "MOYEN", "LOIN"};
const char* TEXTE_ZONE[4] = {"MOINS DE 1 M", "ENTRE 1 ET 3 M", "ENTRE 3 ET 6 M", "PLUS DE 6 M"};
int zone_dist = -1;               // plage affichée actuellement (-1 = aucune)
uint32_t dernier_hist = 0;

// --- Tab5 fixe
uint32_t recus_fixe = 0, dernier_contact = 0;

// ============================================================================ outils de dessin
void police_texte(int taille_px) {
  ecran.setTextSize(1);
  if (taille_px >= 32) ecran.setFont(&fonts::Orbitron_Light_32);
  else ecran.setFont(&fonts::Orbitron_Light_24);
}

void texte(const char* t, int x, int y, int taille_px, uint8_t ancre, uint16_t couleur) {
  police_texte(taille_px);
  ecran.setTextDatum(ancre);
  ecran.setTextColor(couleur);
  ecran.drawString(t, x, y);
}

// Chiffres 7 segments avec les segments « éteints » visibles derrière (effet LCD)
void chiffres(const char* t, int x, int y, float taille, uint8_t ancre) {
  char fantome[16];
  int n = strlen(t);
  for (int i = 0; i < n && i < 15; i++) fantome[i] = (t[i] == '.' || t[i] == ':') ? t[i] : '8';
  fantome[n < 15 ? n : 15] = '\0';
  ecran.setFont(&fonts::Font7);
  ecran.setTextSize(taille);
  ecran.setTextDatum(ancre);
  ecran.setTextColor(C_FANTOME);
  ecran.drawString(fantome, x, y);
  ecran.setTextColor(C_ENCRE);
  ecran.drawString(t, x, y);
}

void cadre(int x, int y, int w, int h) {
  ecran.drawRoundRect(x, y, w, h, 14, C_ENCRE);
  ecran.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 13, C_ENCRE);
  ecran.drawRoundRect(x + 2, y + 2, w - 4, h - 4, 12, C_ENCRE);
}

// Une case de donnée façon GPS : petit titre en haut, valeur en 7 segments, unité à droite
void case_donnee(int x, int y, int w, int h, const char* titre, const char* valeur, const char* unite) {
  cadre(x, y, w, h);
  texte(titre, x + 22, y + 18, 24, top_left, C_ENCRE);
  int droite = unite[0] ? x + w - 100 : x + w - 22;   // pas d'unité : la valeur va jusqu'au bord
  chiffres(valeur, droite, y + h - 22, 1.6f, bottom_right);
  if (unite[0]) texte(unite, x + w - 22, y + h - 26, 24, bottom_right, C_ENCRE);
}

void entete(const char* sous_titre, int barres) {
  texte("TAB-GEOLOC", 30, 30, 32, top_left, C_ENCRE);
  // barres de signal (0 à 5)
  for (int i = 0; i < 5; i++) {
    int bh = 14 + i * 12, bx = W - 160 + i * 26;
    if (i < barres) ecran.fillRect(bx, 82 - bh, 18, bh, C_ENCRE);
    else ecran.drawRect(bx, 82 - bh, 18, bh, C_GRIS);
  }
  ecran.fillRect(30, 100, W - 60, 3, C_ENCRE);
  texte(sous_titre, 30, 120, 24, top_left, C_ENCRE);
  uint32_t s = millis() / 1000;
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60), (unsigned long)(s % 60));
  texte(buf, W - 30, 120, 24, top_right, C_ENCRE);
}

void pied(const char* etat, bool alerte, const char* aide) {
  ecran.fillRect(30, H - 120, W - 60, 3, C_ENCRE);
  int bw = 300;
  if (alerte) {                       // état inversé (encre pleine) quand il y a un problème
    ecran.fillRoundRect(30, H - 95, bw, 60, 10, C_ENCRE);
    texte(etat, 30 + bw / 2, H - 65, 24, middle_center, C_FOND);
  } else {
    ecran.drawRoundRect(30, H - 95, bw, 60, 10, C_ENCRE);
    texte(etat, 30 + bw / 2, H - 65, 24, middle_center, C_ENCRE);
  }
  texte(aide, W - 30, H - 65, 24, middle_right, C_GRIS);
}

int barres_signal(float rssi) {
  if (isnan(rssi)) return 0;
  if (rssi > -55) return 5;
  if (rssi > -65) return 4;
  if (rssi > -75) return 3;
  if (rssi > -85) return 2;
  return 1;
}

void afficher() { ecran.pushSprite(0, 0); }

// ============================================================================ Wi-Fi
void ecran_attente(const char* ligne1, const char* ligne2) {
  ecran.fillScreen(C_FOND);
  entete("DEMARRAGE", 0);
  cadre(30, 180, W - 60, 420);
  texte(ligne1, W / 2, 330, 32, middle_center, C_ENCRE);
  texte(ligne2, W / 2, 400, 24, middle_center, C_ENCRE);
  pied("RECHERCHE", true, "");
  afficher();
}

void demarrer_wifi() {
  WiFi.setPins(SDIO_CLK, SDIO_CMD, SDIO_D0, SDIO_D1, SDIO_D2, SDIO_D3, SDIO_RST);
#if ROLE == 1 && MODE_PARTAGE == 0
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_MDP);
#else
  const char* ssid = MODE_PARTAGE ? PARTAGE_SSID : AP_SSID;
  const char* mdp  = MODE_PARTAGE ? PARTAGE_MDP  : AP_MDP;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, mdp);
  while (WiFi.status() != WL_CONNECTED) {
    ecran_attente("CONNEXION", ssid);
    delay(300);
  }
#endif
  udp.begin(PORT);
}

IPAddress adresse_fixe() {
#if MODE_PARTAGE == 0
  return IPAddress(192, 168, 4, 1);
#else
  return IPAddress(255, 255, 255, 255);
#endif
}

// ============================================================================ setup
void setup() {
  M5.begin();
  M5.Speaker.setVolume(200);
  Serial.begin(115200);
  if (M5.Display.width() > M5.Display.height()) M5.Display.setRotation(M5.Display.getRotation() ^ 1);  // portrait
  W = M5.Display.width();
  H = M5.Display.height();
  ecran.setColorDepth(16);
  ecran.setPsram(true);
  ecran.createSprite(W, H);
  C_FOND    = ecran.color565(186, 196, 160);
  C_ENCRE   = ecran.color565(30, 40, 32);
  C_FANTOME = ecran.color565(172, 182, 146);
  C_GRIS    = ecran.color565(120, 132, 104);
  ecran_attente("DEMARRAGE", "TAB5");
  demarrer_wifi();
#if ROLE == 2
  Serial.println("temps_ms;rssi_dBm;distance_m;envoyes;recus;perte_pct;rtt_moy_ms");
#endif
}

// ============================================================================ Tab5 FIXE
void dessiner_fixe() {
  bool actif = dernier_contact && (millis() - dernier_contact < 2000);
  ecran.fillScreen(C_FOND);
  entete("MODE : BALISE FIXE", 5);

  // grand cadre : ondes radio qui s'élargissent
  int cx = W / 2, cy = 380;
  cadre(30, 180, W - 60, 440);
  ecran.fillCircle(cx, cy, 18, C_ENCRE);
  for (int k = 0; k < 3; k++) {
    int r = 40 + ((millis() / 12 + k * 40) % 120);
    uint16_t c = actif ? C_ENCRE : C_GRIS;
    ecran.drawCircle(cx, cy, r, c);
    ecran.drawCircle(cx, cy, r + 1, c);
  }
  texte(actif ? "EMISSION EN COURS" : "EN ATTENTE DU MOBILE", W / 2, 585, 24, middle_center, C_ENCRE);

  // infos réseau
  cadre(30, 650, W - 60, 150);
  texte("RESEAU", 52, 670, 24, top_left, C_ENCRE);
  texte(MODE_PARTAGE ? PARTAGE_SSID : AP_SSID, 52, 712, 32, top_left, C_ENCRE);
  IPAddress ip = MODE_PARTAGE ? WiFi.localIP() : WiFi.softAPIP();
  texte(ip.toString().c_str(), W - 52, 760, 24, bottom_right, C_GRIS);

  // compteurs
  int cw = (W - 80) / 2;
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)(recus_fixe % 10000));
  case_donnee(30, 830, cw, 200, "MESSAGES", buf, "");
  uint32_t s = dernier_contact ? (millis() - dernier_contact) / 1000 : 0;
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)(s > 999 ? 999 : s));
  case_donnee(50 + cw, 830, cw, 200, "DERNIER", dernier_contact ? buf : "--", "s");

  pied(actif ? "LIEN OK" : "EN ATTENTE", !actif, "");
  afficher();
}

void boucle_fixe() {
  int taille = udp.parsePacket();
  if (taille > 0) {
    int n = udp.read(buf, sizeof(buf) - 1);
    buf[n] = '\0';
    if (strncmp(buf, "PING;", 5) == 0) {
      buf[1] = 'O';                         // PING -> PONG (même numéro, même heure d'envoi)
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write((const uint8_t*)buf, n);
      udp.endPacket();
      recus_fixe++;
      dernier_contact = millis();
    }
  }
  if (millis() - dernier_dessin > 100) {    // animation fluide (10 images/s)
    dernier_dessin = millis();
    dessiner_fixe();
  }
}

// ============================================================================ Tab5 MOBILE
float distance_estimee(float rssi) {        // modèle : RSSI = RSSI_1M - 10·n·log10(d)
  if (isnan(rssi)) return NAN;
  float d = powf(10.0f, (RSSI_1M - rssi) / (10.0f * N_PROPAG));
  return d < 0.1f ? 0.1f : (d > 99.9f ? 99.9f : d);
}

int tendance() {                            // +1 rapprochement, -1 éloignement, 0 stable
  if (nb_hist < N_HIST) return 0;
  float ancien = historique[idx_hist];      // valeur d'il y a 3 s
  float ecart = rssi_lisse - ancien;
  if (ecart >= 4.0f) return 1;              // il faut au moins 4 dB d'écart pour changer
  if (ecart <= -4.0f) return -1;
  return 0;
}

float mediane() {                           // médiane des dernières mesures brutes
  float t[N_MED];
  for (int i = 0; i < nb_brut; i++) t[i] = brut[i];
  for (int i = 1; i < nb_brut; i++) {       // tri par insertion
    float v = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return t[nb_brut / 2];
}

int zone_distance(float d) {                // plage de distance avec hystérésis (marge de 30 %)
  if (isnan(d)) return -1;
  int z = 0;
  while (z < 3 && d > BORNES[z]) z++;
  if (zone_dist < 0) return z;
  if (z > zone_dist && d < BORNES[zone_dist] * 1.3f) return zone_dist;     // pas assez loin pour changer
  if (z < zone_dist && d > BORNES[zone_dist - 1] * 0.7f) return zone_dist; // pas assez près pour changer
  return z;
}

void dessiner_mobile(bool lien) {
  float d = lien ? distance_estimee(rssi_lisse) : NAN;
  int tend = lien ? tendance() : 0;
  float perte = envoyes ? 100.0f * (envoyes - recus) / envoyes : 0;
  uint32_t rtt = recus ? somme_rtt / recus : 0;

  ecran.fillScreen(C_FOND);
  entete("CIBLE : TAB5 FIXE", lien ? barres_signal(rssi_lisse) : 0);

  // --- distance
  cadre(30, 180, W - 60, 420);
  texte("DISTANCE", 52, 200, 24, top_left, C_ENCRE);
  zone_dist = lien ? zone_distance(d) : -1;
  police_texte(32);
  ecran.setTextSize(2);                     // nom de la plage en grand
  ecran.setTextDatum(middle_center);
  ecran.setTextColor(C_ENCRE);
  ecran.drawString(zone_dist < 0 ? "---" : NOM_ZONE[zone_dist], W / 2, 315);
  texte(zone_dist < 0 ? "AUCUNE MESURE" : TEXTE_ZONE[zone_dist], W / 2, 405, 32, middle_center, C_ENCRE);

  // jauge : 4 blocs, plus on est proche plus il y en a d'allumés
  int gx = 60, gy = 470, gw = W - 120, bw = gw / 4;
  for (int i = 0; i < 4; i++) {
    bool allume = zone_dist >= 0 && (3 - i) >= zone_dist;
    ecran.fillRect(gx + i * bw + 6, gy, bw - 12, 50, allume ? C_ENCRE : C_FANTOME);
  }
  texte("LOIN", gx, gy + 58, 24, top_left, C_GRIS);
  texte("PROCHE", gx + gw, gy + 58, 24, top_right, C_GRIS);

  // --- tendance
  cadre(30, 620, W - 60, 120);
  int ax = 110, ay = 680;
  if (tend > 0) ecran.fillTriangle(ax, ay - 40, ax - 36, ay + 30, ax + 36, ay + 30, C_ENCRE);
  else if (tend < 0) ecran.fillTriangle(ax, ay + 40, ax - 36, ay - 30, ax + 36, ay - 30, C_ENCRE);
  else ecran.fillCircle(ax, ay, 22, C_ENCRE);
  texte(!lien ? "AUCUNE CIBLE" : (tend > 0 ? "RAPPROCHEMENT" : (tend < 0 ? "ELOIGNEMENT" : "STABLE")),
        180, ay, 32, middle_left, C_ENCRE);

  // --- 4 cases de données
  int cw = (W - 80) / 2, ch = 155;
  if (lien && !isnan(rssi_lisse)) {
    int rs = (int)roundf(rssi_lisse);
    rs = rs > -20 ? -20 : (rs < -99 ? -99 : rs);          // toujours 3 caractères maximum
    snprintf(buf, sizeof(buf), "%d", rs);
  } else strcpy(buf, "---");
  case_donnee(30, 760, cw, ch, "SIGNAL", buf, "dBm");
  snprintf(buf, sizeof(buf), "%.1f", perte);
  case_donnee(50 + cw, 760, cw, ch, "PERTE", buf, "%");
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)(rtt > 999 ? 999 : rtt));
  case_donnee(30, 935, cw, ch, "LATENCE", buf, "ms");
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)(recus % 10000));
  case_donnee(50 + cw, 935, cw, ch, "RECUS", buf, "");

  pied(lien ? "LIEN OK" : "LIEN PERDU", !lien, "TOUCHER = RAZ");
  afficher();
}

void boucle_mobile() {
  // 1) PING toutes les 200 ms
  if (millis() - dernier_ping >= PERIODE_PING_MS) {
    dernier_ping = millis();
    int n = snprintf(buf, sizeof(buf), "PING;%lu;%lu", (unsigned long)numero++, (unsigned long)millis());
    udp.beginPacket(adresse_fixe(), PORT);
    udp.write((const uint8_t*)buf, n);
    udp.endPacket();
    envoyes++;
  }
  // 2) réponses PONG
  int taille = udp.parsePacket();
  if (taille > 0) {
    int n = udp.read(buf, sizeof(buf) - 1);
    buf[n] = '\0';
    unsigned long num, t_envoi;
    if (sscanf(buf, "PONG;%lu;%lu", &num, &t_envoi) == 2) {
      recus++;
      somme_rtt += millis() - t_envoi;
      dernier_pong = millis();
    }
  }
  bool lien = (WiFi.status() == WL_CONNECTED) && dernier_pong && (millis() - dernier_pong < 2000);

  // 3) RSSI lissé + historique pour la tendance
  if (millis() - dernier_hist >= 200) {
    dernier_hist = millis();
    if (WiFi.status() == WL_CONNECTED) {
      int r = WiFi.RSSI();
      if (r < 0 && r > -100) {              // on ignore les lectures fausses (0, valeurs positives, -127…)
        brut[idx_brut] = r;
        idx_brut = (idx_brut + 1) % N_MED;
        if (nb_brut < N_MED) nb_brut++;
        float med = mediane();
        rssi_lisse = isnan(rssi_lisse) ? med : 0.85f * rssi_lisse + 0.15f * med;
        historique[idx_hist] = rssi_lisse;
        idx_hist = (idx_hist + 1) % N_HIST;
        if (nb_hist < N_HIST) nb_hist++;
      }
    }
  }
  // 4) toucher = remise à zéro
  if (M5.Touch.getCount() > 0 && M5.Touch.getDetail().wasPressed()) {
    envoyes = recus = somme_rtt = 0;
    M5.Speaker.tone(2000, 100);
  }
  // 5) affichage (5 images/s)
  if (millis() - dernier_dessin > 200) {
    dernier_dessin = millis();
    dessiner_mobile(lien);
  }
  // 6) une ligne CSV par seconde (moniteur série)
  if (millis() - dernier_csv >= 1000) {
    dernier_csv = millis();
    float perte = envoyes ? 100.0f * (envoyes - recus) / envoyes : 0;
    Serial.printf("%lu;%.1f;%.2f;%lu;%lu;%.1f;%lu\n", (unsigned long)millis(), rssi_lisse, distance_estimee(rssi_lisse),
                  (unsigned long)envoyes, (unsigned long)recus, perte, (unsigned long)(recus ? somme_rtt / recus : 0));
  }
}

void loop() {
  M5.update();
#if ROLE == 1
  boucle_fixe();
#else
  boucle_mobile();
#endif
}
