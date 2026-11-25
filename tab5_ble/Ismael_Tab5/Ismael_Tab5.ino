/*
  >>> ISMAEL – Tab5 (carte M5Tab5). Garder voix.h dans le MÊME dossier que ce fichier.
  TAB-Géoloc – Tab5 : écoute des balises BLE + bip « radar de recul » + DIRECTION de l'obstacle + VOIX
  ---------------------------------------------------------------------------------------------------
  Carte        : "M5Tab5" (paquet M5Stack 3.3.x ou "esp32 by Espressif" 3.3.1 ou plus)
  Bibliothèques: M5Unified >= 0.2.23, M5GFX >= 0.2.30

  Comment le Tab5 trouve la direction (une seule antenne ne donne pas la direction toute seule) :
    - le Tab5 est porté sur le torse : le CORPS de l'usager fait écran, donc le signal est plus fort
      quand l'usager est TOURNÉ VERS la balise ;
    - le GYROSCOPE du Tab5 mesure de combien l'usager a tourné (angle) ;
    - quand l'usager tourne doucement sur lui-même, le Tab5 retient l'angle où le RSSI était le plus fort
      et le compare à l'angle actuel -> "Obstacle à gauche / à droite / devant / derrière".
    - si l'usager n'a pas encore tourné assez, la voix dit "Obstacle proche. Tournez-vous lentement."

  Le bip continue : plus on s'approche, plus il est FORT, RAPIDE et AIGU (il se tait pendant la voix).
  Toucher l'écran = couper / remettre le son.
  Au démarrage : poser le Tab5 IMMOBILE 2 secondes (réglage du gyroscope).
*/
#include <M5Unified.h>
#include <WiFi.h>  // seulement pour WiFi.setPins (liaison SDIO vers la puce radio ESP32-C6)
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "voix.h"

// ============================================================ RÉGLAGES
struct Balise { uint8_t type; uint8_t num; const char* nom; float rssi_1m; };
const int NB_BALISES = 2;
Balise BALISES[NB_BALISES] = {
  // type (1 = zone, 2 = obstacle), numéro, nom affiché, RSSI mesuré à 1 m (À CALIBRER)
  {1, 1, "ZONE 1",     -60.0f},   // ESP-32  (antenne imprimée)
  {2, 1, "OBSTACLE 1", -56.0f},   // ESP-32U (antenne externe)
};
const float N_PROPAG        = 2.5f;   // 2 = espace libre, 3 = intérieur encombré
const float PORTEE_BIP_M    = 8.0f;   // le bip commence en dessous de cette distance
const float SEUIL_OBSTACLE  = 1.5f;   // alerte obstacle à l'écran (m)
const float SEUIL_VOIX_M    = 3.0f;   // la voix parle d'un obstacle à moins de 3 m
const uint32_t PERTE_MS     = 5000;   // "Signal perdu" après 5 s sans balise

// --- voix ---
const uint32_t PAUSE_VOIX_MS   = 4000;   // au moins 4 s entre deux phrases (pas de spam)
const uint32_t REPETE_MEME_MS  = 8000;   // la même phrase au plus toutes les 8 s

// --- direction ---
const float SENS_ROTATION    = 1.0f;     // mettre -1.0f si GAUCHE et DROITE sont inversées au test
const float BALAYAGE_MIN_DEG = 180.0f;   // l'usager doit faire au moins un demi-tour
const uint32_t FENETRE_DIR_MS = 10000;   // on regarde les 10 dernières secondes
const float DEVANT_DEG       = 30.0f;    // |angle| < 30° -> devant
const float DERRIERE_DEG     = 135.0f;   // |angle| > 135° -> derrière

const uint8_t UUID_TABGEO[16] = {0x7A, 0xB0, 0x6E, 0x01, 0x5E, 0xE5, 0x40, 0x26,
                                 0x9A, 0x1B, 0x15, 0x7E, 0x5E, 0xE5, 0x20, 0x26};

// canaux du haut-parleur : la voix et le bip ne se coupent pas l'un l'autre
const int CANAL_VOIX = 0;
const int CANAL_BIP  = 1;

// ============================================================ DONNÉES PARTAGÉES AVEC LE BLUETOOTH
portMUX_TYPE verrou = portMUX_INITIALIZER_UNLOCKED;
const int TAILLE_FILE = 16;
int16_t  file_rssi[NB_BALISES][TAILLE_FILE];
uint8_t  nb_file[NB_BALISES];
uint32_t vu_ms[NB_BALISES];

class EcouteBalises : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (!dev.haveManufacturerData()) return;
    String m = dev.getManufacturerData();
    if (m.length() != 25) return;
    const uint8_t* d = (const uint8_t*)m.c_str();
    if (d[0] != 0x4C || d[1] != 0x00 || d[2] != 0x02 || d[3] != 0x15) return;  // pas un iBeacon
    if (memcmp(d + 4, UUID_TABGEO, 16) != 0) return;                           // pas une de nos balises
    for (int i = 0; i < NB_BALISES; i++) {
      if (BALISES[i].type == d[21] && BALISES[i].num == d[23]) {
        portENTER_CRITICAL(&verrou);
        if (nb_file[i] < TAILLE_FILE) file_rssi[i][nb_file[i]++] = dev.getRSSI();
        vu_ms[i] = millis();
        portEXIT_CRITICAL(&verrou);
      }
    }
  }
};

// ============================================================ TRAITEMENT DU RSSI (par balise)
const int N_MED  = 5;    // médiane des 5 dernières trames
const int N_HIST = 15;   // 3 s d'historique pour la tendance
const int N_DIR  = 100;  // 10 s de couples (angle, RSSI), 1 toutes les 100 ms
struct Etat {
  float brut[N_MED]; int nb, idx;
  float rapide;                   // médiane seule (réagit vite) -> sert pour la direction
  float lisse;                    // médiane + lissage -> sert pour la distance et le bip
  float hist[N_HIST]; int nh, ih;
  float dir_angle[N_DIR], dir_rssi[N_DIR]; uint32_t dir_t[N_DIR]; int nd, id;
  int dernier;
  bool actif;
};
Etat E[NB_BALISES];

void remise_a_zero(int i) {
  E[i].nb = E[i].idx = E[i].nh = E[i].ih = E[i].nd = E[i].id = 0;
  E[i].rapide = E[i].lisse = NAN;
  E[i].dernier = 0;
  E[i].actif = false;
}

float mediane(const float* v, int n) {
  float t[N_MED];
  for (int i = 0; i < n; i++) t[i] = v[i];
  for (int i = 1; i < n; i++) {
    float x = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; }
    t[j + 1] = x;
  }
  return t[n / 2];
}

void ajouter_mesure(int i, int r) {
  if (r >= 0 || r < -100) return;
  Etat& e = E[i];
  e.brut[e.idx] = r;
  e.idx = (e.idx + 1) % N_MED;
  if (e.nb < N_MED) e.nb++;
  e.rapide = mediane(e.brut, e.nb);
  e.lisse = isnan(e.lisse) ? e.rapide : 0.7f * e.lisse + 0.3f * e.rapide;
  e.dernier = r;
}

float distance_m(int i) {
  float d = powf(10.0f, (BALISES[i].rssi_1m - E[i].lisse) / (10.0f * N_PROPAG));
  return constrain(d, 0.1f, 50.0f);
}

int tendance(int i) {
  if (E[i].nh < N_HIST) return 0;
  float ecart = E[i].lisse - E[i].hist[E[i].ih];
  if (ecart > 4.0f) return 1;
  if (ecart < -4.0f) return -1;
  return 0;
}

// ============================================================ GYROSCOPE : angle de rotation de l'usager
bool imu_ok = false;
float biais[3] = {0, 0, 0};   // dérive du gyroscope mesurée au démarrage
float angle = 0;              // angle de rotation (degrés, + = vers la gauche)
uint32_t dernier_imu_us = 0;

void calibrer_gyro() {
  float s[3] = {0, 0, 0};
  int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    if (M5.Imu.update()) {
      float gx, gy, gz;
      M5.Imu.getGyro(&gx, &gy, &gz);
      s[0] += gx; s[1] += gy; s[2] += gz; n++;
    }
    delay(5);
  }
  if (n > 0) for (int k = 0; k < 3; k++) biais[k] = s[k] / n;
}

void lire_gyro() {
  if (!imu_ok || !M5.Imu.update()) return;
  float ax, ay, az, gx, gy, gz;
  M5.Imu.getAccel(&ax, &ay, &az);   // gravité (en g) -> donne la verticale
  M5.Imu.getGyro(&gx, &gy, &gz);    // vitesse de rotation (°/s)
  gx -= biais[0]; gy -= biais[1]; gz -= biais[2];
  float na = sqrtf(ax * ax + ay * ay + az * az);
  uint32_t maintenant = micros();
  float dt = (maintenant - dernier_imu_us) / 1e6f;
  dernier_imu_us = maintenant;
  if (na < 0.5f || dt <= 0 || dt > 0.2f) return;
  // vitesse de rotation autour de la verticale, quelle que soit la façon dont le Tab5 est tenu
  float vitesse = SENS_ROTATION * (gx * ax + gy * ay + gz * az) / na;
  angle += vitesse * dt;
}

float ramener_180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

const float AMPLITUDE_MIN = 3.0f;   // écart minimum (dB) entre "face à la balise" et "dos à la balise"

float det3(double a, double b, double c, double d, double e, double f, double g, double h, double i) {
  return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

// direction de la balise i : on ajuste une courbe RSSI = A + B.cos(angle) + C.sin(angle)
// sur toutes les mesures du tour, puis on prend l'angle où cette courbe est maximale.
bool direction(int i, float& relatif) {
  Etat& e = E[i];
  float amin = 1e9, amax = -1e9;
  double n = 0, sc = 0, ss = 0, cc = 0, cs = 0, s2 = 0, r = 0, rc = 0, rs = 0;
  for (int k = 0; k < e.nd; k++) {
    if (millis() - e.dir_t[k] > FENETRE_DIR_MS) continue;
    double th = e.dir_angle[k] * DEG_TO_RAD, c = cos(th), s = sin(th), y = e.dir_rssi[k];
    n++; sc += c; ss += s; cc += c * c; cs += c * s; s2 += s * s;
    r += y; rc += y * c; rs += y * s;
    amin = min(amin, e.dir_angle[k]);
    amax = max(amax, e.dir_angle[k]);
  }
  if (n < 20 || amax - amin < BALAYAGE_MIN_DEG) return false;
  float D = det3(n, sc, ss, sc, cc, cs, ss, cs, s2);
  if (fabsf(D) < 1e-6f) return false;
  float B = det3(n, r, ss, sc, rc, cs, ss, rs, s2) / D;
  float C = det3(n, sc, r, sc, cc, rc, ss, cs, rs) / D;
  if (sqrtf(B * B + C * C) < AMPLITUDE_MIN) return false;   // pas de côté nettement plus fort
  relatif = ramener_180(atan2f(C, B) * RAD_TO_DEG - angle);
  return true;
}

// ============================================================ SON : voix + bip
bool son = true;
uint32_t dernier_bip = 0;
uint32_t derniere_voix = 0, derniere_meme = 0;
int phrase_precedente = -1;

enum { V_GAUCHE, V_DROITE, V_DEVANT, V_DERRIERE, V_TOURNEZ, V_PERDU };
const uint8_t* VOIX[]  = {voix_gauche, voix_droite, voix_devant, voix_derriere, voix_tournez, voix_perdu};
const size_t   TAILLE[] = {sizeof(voix_gauche), sizeof(voix_droite), sizeof(voix_devant),
                           sizeof(voix_derriere), sizeof(voix_tournez), sizeof(voix_perdu)};
const char*    TEXTE[] = {"A GAUCHE", "A DROITE", "DEVANT", "DERRIERE", "TOURNEZ-VOUS", "SIGNAL PERDU"};

bool voix_en_cours() { return M5.Speaker.isPlaying(CANAL_VOIX); }

// parle seulement si : son activé, pas déjà en train de parler, 4 s depuis la dernière phrase,
// et 8 s si c'est la même phrase que la fois d'avant
void parler_base(int p, bool forcer) {
  if (!son || voix_en_cours()) return;
  uint32_t t = millis();
  if (!forcer && t - derniere_voix < PAUSE_VOIX_MS) return;
  if (!forcer && p == phrase_precedente && t - derniere_meme < REPETE_MEME_MS) return;
  M5.Speaker.playWav(VOIX[p], TAILLE[p], 1, CANAL_VOIX, true);
  derniere_voix = t;
  if (p != phrase_precedente || t - derniere_meme >= REPETE_MEME_MS) derniere_meme = t;
  phrase_precedente = p;
}
void parler(int p)       { parler_base(p, false); }   // respecte les pauses
void parler_force(int p) { parler_base(p, true); }    // pour "Signal perdu" (une seule fois)

void bip_radar(int i, float d) {
  if (!son || d > PORTEE_BIP_M || voix_en_cours()) return;   // le bip se tait pendant la voix
  float p = log10f(PORTEE_BIP_M / d) / log10f(PORTEE_BIP_M / 0.5f);
  p = constrain(p, 0.0f, 1.0f);
  uint32_t intervalle = 1200 - (uint32_t)(p * 1100);
  if (millis() - dernier_bip < intervalle) return;
  dernier_bip = millis();
  float f0 = (BALISES[i].type == 2) ? 1200.0f : 600.0f;
  M5.Speaker.setChannelVolume(CANAL_BIP, (uint8_t)(60 + p * 195));
  M5.Speaker.tone(f0 * (1.0f + p), 60, CANAL_BIP);
}

// ============================================================ AFFICHAGE
M5Canvas ecran(&M5.Display);
int W, H;
int dir_affichee = -1;   // dernière direction calculée (pour l'écran)

const char* plage(float d) {
  if (d < 1.0f) return "TRES PROCHE";
  if (d < 3.0f) return "PROCHE";
  if (d < 6.0f) return "MOYEN";
  return "LOIN";
}

void dessiner(int proche, bool perdu) {
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(top_center);
  ecran.setTextSize(1);
  ecran.setFont(&fonts::FreeSansBold12pt7b);
  ecran.setTextColor(TFT_DARKGREY);
  ecran.drawString("TAB-GEOLOC  -  BALISES BLE", W / 2, 30);

  if (proche < 0) {
    ecran.setFont(&fonts::FreeSansBold24pt7b);
    ecran.setTextSize(1.5f);
    ecran.setTextColor(perdu ? TFT_ORANGE : TFT_WHITE);
    ecran.drawString(perdu ? "SIGNAL PERDU" : "RECHERCHE...", W / 2, H / 2 - 120);
  } else {
    float d = distance_m(proche);
    bool obstacle = BALISES[proche].type == 2;
    uint16_t c = obstacle ? TFT_RED : TFT_GREEN;
    ecran.setFont(&fonts::FreeSansBold24pt7b);
    ecran.setTextSize(1.5f);
    ecran.setTextColor(c);
    ecran.drawString(BALISES[proche].nom, W / 2, 130);
    ecran.setTextSize(1.2f);
    ecran.setTextColor(TFT_WHITE);
    ecran.drawString(plage(d), W / 2, 250);
    char txt[32];
    snprintf(txt, sizeof(txt), "~ %.1f m", d);
    ecran.setTextSize(2.0f);
    ecran.drawString(txt, W / 2, 340);

    float p = constrain(log10f(PORTEE_BIP_M / d) / log10f(PORTEE_BIP_M / 0.5f), 0.0f, 1.0f);
    int bx = 60, by = 510, bw = W - 120, bh = 50;
    ecran.drawRect(bx, by, bw, bh, TFT_WHITE);
    ecran.fillRect(bx + 4, by + 4, (int)((bw - 8) * p), bh - 8, c);

    int t = tendance(proche);
    ecran.setTextSize(1.0f);
    ecran.drawString(t > 0 ? "RAPPROCHEMENT" : (t < 0 ? "ELOIGNEMENT" : "STABLE"), W / 2, 600);

    if (obstacle) {
      ecran.setTextColor(TFT_YELLOW);
      ecran.drawString(dir_affichee >= 0 ? TEXTE[dir_affichee] : "DIRECTION ?", W / 2, 690);
    }
    if (obstacle && d < SEUIL_OBSTACLE) {
      ecran.fillRect(0, 790, W, 110, TFT_RED);
      ecran.setTextColor(TFT_WHITE);
      ecran.drawString("ATTENTION OBSTACLE", W / 2, 820);
    }
  }

  ecran.setFont(&fonts::FreeSans12pt7b);
  ecran.setTextSize(1.0f);
  ecran.setTextDatum(top_left);
  for (int i = 0; i < NB_BALISES; i++) {
    char ligne[64];
    if (E[i].actif && !isnan(E[i].lisse))
      snprintf(ligne, sizeof(ligne), "%-11s %4.0f dBm   %4.1f m", BALISES[i].nom, E[i].lisse, distance_m(i));
    else
      snprintf(ligne, sizeof(ligne), "%-11s  pas de signal", BALISES[i].nom);
    ecran.setTextColor(E[i].actif ? TFT_LIGHTGREY : TFT_DARKGREY);
    ecran.drawString(ligne, 60, H - 260 + i * 45);
  }
  char ang[48];
  if (imu_ok) snprintf(ang, sizeof(ang), "Rotation : %+.0f deg", ramener_180(angle));
  else snprintf(ang, sizeof(ang), "Gyroscope non detecte");
  ecran.setTextColor(TFT_DARKGREY);
  ecran.drawString(ang, 60, H - 160);
  ecran.setTextDatum(top_center);
  ecran.drawString(son ? "SON : OUI  (toucher l'ecran pour couper)" : "SON : COUPE  (toucher l'ecran)", W / 2, H - 70);
  ecran.pushSprite(0, 0);
}

// ============================================================ SETUP / LOOP
BLEScan* scan;

void setup() {
  M5.begin();
  Serial.begin(115200);
  if (M5.Display.width() > M5.Display.height()) M5.Display.setRotation(M5.Display.getRotation() ^ 1);
  W = M5.Display.width();
  H = M5.Display.height();
  ecran.setColorDepth(16);
  ecran.setPsram(true);
  ecran.createSprite(W, H);
  for (int i = 0; i < NB_BALISES; i++) { remise_a_zero(i); nb_file[i] = 0; vu_ms[i] = 0; }

  M5.Speaker.setVolume(220);                     // volume général
  M5.Speaker.setChannelVolume(CANAL_VOIX, 255);  // voix toujours à fond (dans le volume général)

  // réglage du gyroscope : le Tab5 doit rester immobile
  ecran.fillScreen(TFT_BLACK);
  ecran.setFont(&fonts::FreeSansBold18pt7b);
  ecran.setTextDatum(middle_center);
  ecran.setTextColor(TFT_WHITE);
  ecran.drawString("Ne bougez pas (2 s)...", W / 2, H / 2);
  ecran.pushSprite(0, 0);
  imu_ok = M5.Imu.isEnabled();
  if (imu_ok) calibrer_gyro();
  dernier_imu_us = micros();

  WiFi.setPins(12, 13, 11, 10, 9, 8, 15);
  BLEDevice::init("");
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new EcouteBalises(), true);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(99);
  scan->start(0, nullptr, false);

  Serial.println("temps_ms;balise;rssi_brut;rssi_lisse;distance_m;angle_deg");
}

uint32_t dernier_hist = 0, dernier_dir = 0, dernier_dessin = 0, dernier_csv = 0, dernier_controle = 0, derniere_balise = 0;
bool deja_perdu = false;

void loop() {
  M5.update();
  lire_gyro();

  // 1) récupérer les mesures déposées par le Bluetooth
  for (int i = 0; i < NB_BALISES; i++) {
    int16_t copie[TAILLE_FILE];
    uint8_t n;
    uint32_t vu;
    portENTER_CRITICAL(&verrou);
    n = nb_file[i];
    memcpy(copie, file_rssi[i], n * sizeof(int16_t));
    nb_file[i] = 0;
    vu = vu_ms[i];
    portEXIT_CRITICAL(&verrou);
    for (int k = 0; k < n; k++) ajouter_mesure(i, copie[k]);
    bool actif = vu && (millis() - vu < PERTE_MS);
    if (!actif && E[i].actif) remise_a_zero(i);
    E[i].actif = actif && !isnan(E[i].lisse);
  }

  // 2) historique pour la tendance (200 ms) et couples angle/RSSI pour la direction (100 ms)
  if (millis() - dernier_hist >= 200) {
    dernier_hist = millis();
    for (int i = 0; i < NB_BALISES; i++) {
      if (!E[i].actif) continue;
      E[i].hist[E[i].ih] = E[i].lisse;
      E[i].ih = (E[i].ih + 1) % N_HIST;
      if (E[i].nh < N_HIST) E[i].nh++;
    }
  }
  if (millis() - dernier_dir >= 100) {
    dernier_dir = millis();
    for (int i = 0; i < NB_BALISES; i++) {
      if (!E[i].actif) continue;
      Etat& e = E[i];
      e.dir_angle[e.id] = angle;
      e.dir_rssi[e.id] = e.rapide;
      e.dir_t[e.id] = millis();
      e.id = (e.id + 1) % N_DIR;
      if (e.nd < N_DIR) e.nd++;
    }
  }

  // 3) balise la plus proche
  int proche = -1;
  float dmin = 1e9;
  for (int i = 0; i < NB_BALISES; i++) {
    if (!E[i].actif) continue;
    float d = distance_m(i);
    if (d < dmin) { dmin = d; proche = i; }
  }

  // 4) bip radar (toujours) + voix pour la direction de l'obstacle le plus proche
  int obst = -1;
  float dobst = 1e9;
  for (int i = 0; i < NB_BALISES; i++)
    if (E[i].actif && BALISES[i].type == 2 && distance_m(i) < dobst) { dobst = distance_m(i); obst = i; }

  if (proche >= 0) {
    derniere_balise = millis();
    deja_perdu = false;
    bip_radar(proche, dmin);
  }
  dir_affichee = -1;
  if (obst >= 0) {
    float rel;
    if (imu_ok && direction(obst, rel)) {
      if (fabsf(rel) <= DEVANT_DEG) dir_affichee = V_DEVANT;
      else if (fabsf(rel) >= DERRIERE_DEG) dir_affichee = V_DERRIERE;
      else dir_affichee = (rel > 0) ? V_GAUCHE : V_DROITE;
    }
    if (dobst < SEUIL_VOIX_M) parler(dir_affichee >= 0 ? dir_affichee : V_TOURNEZ);
  }

  bool perdu = (proche < 0) && derniere_balise;
  if (perdu && !deja_perdu) {
    deja_perdu = true;
    parler_force(V_PERDU);
  }

  // 5) toucher l'écran = son oui / non
  if (M5.Touch.getCount() > 0 && M5.Touch.getDetail().wasPressed()) {
    son = !son;
    if (!son) M5.Speaker.stop();
    M5.Speaker.setChannelVolume(CANAL_BIP, 150);
    M5.Speaker.tone(son ? 1500 : 500, 80, CANAL_BIP);
  }

  // 6) affichage (5 images/s)
  if (millis() - dernier_dessin > 200) {
    dernier_dessin = millis();
    dessiner(proche, perdu);
  }

  // 7) CSV toutes les 0,5 s (moniteur série)
  if (millis() - dernier_csv >= 500) {
    dernier_csv = millis();
    for (int i = 0; i < NB_BALISES; i++)
      if (E[i].actif)
        Serial.printf("%lu;%s;%d;%.1f;%.2f;%.0f\n", (unsigned long)millis(), BALISES[i].nom,
                      E[i].dernier, E[i].lisse, distance_m(i), angle);
  }

  // 8) relancer l'écoute si elle s'est arrêtée
  if (millis() - dernier_controle >= 5000) {
    dernier_controle = millis();
    if (!scan->isScanning()) scan->start(0, nullptr, false);
    scan->clearResults();
  }
}
