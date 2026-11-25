/*
  TAB-Géoloc – Tab5 : DÉCISION DU NIVEAU DE DANGER PAR RÉSEAU DE NEURONES
  -----------------------------------------------------------------------
  Carte        : "M5Tab5" (paquet M5Stack 3.3.x ou "esp32 by Espressif" 3.3.1 ou plus)
  Bibliothèques: M5Unified >= 0.2.23, M5GFX >= 0.2.30
  Fichiers à garder dans le MÊME dossier :
    - modele.h : le réseau entraîné (créé par "python analyse_reseau.py analyse")
    - voix.h   : les phrases en français

  Toutes les 200 ms :
    1. le Tab5 mesure les 4 infos : det1, rssi1 (balise ZONE), det2, rssi2 (balise OBSTACLE)
       (+ heure et température si le réseau choisi les utilise)
    2. le réseau de neurones calcule la probabilité de chaque niveau : LOIN / PROCHE / TRES PROCHE
    3. on fait la moyenne sur 1 s (5 décisions) pour éviter que ça clignote
    4. le Tab5 prévient l'usager :
         LOIN        -> silence
         PROCHE      -> bip moyen + « Attention, obstacle proche. »
         TRES PROCHE -> bip rapide et fort + « Danger ! Obstacle très proche ! »
  La voix attend au moins 4 s entre deux phrases, et 8 s avant de répéter la même.
  Toucher l'écran = couper / remettre le son.
*/
#include <M5Unified.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "modele.h"
#include "voix.h"

// ============================================================ RÉGLAGES (identiques à Tab5_Collecte)
const uint8_t UUID_TABGEO[16] = {0x7A, 0xB0, 0x6E, 0x01, 0x5E, 0xE5, 0x40, 0x26,
                                 0x9A, 0x1B, 0x15, 0x7E, 0x5E, 0xE5, 0x20, 0x26};
const int NB_BALISES = 2;
const uint8_t TYPE[NB_BALISES] = {1, 2};          // balise 1 = ZONE, balise 2 = OBSTACLE
const uint8_t NUM[NB_BALISES]  = {1, 1};
const uint32_t FENETRE_DETECTION_MS = 1000;
const float RSSI_ABSENT = -100.0f;
const uint32_t PERIODE_MS = 200;
const uint32_t PERTE_MS = 5000;                   // "Signal perdu" après 5 s sans aucune balise
const int N_VOTE = 5;                             // moyenne des probabilités sur 5 décisions (1 s)
const uint32_t PAUSE_VOIX_MS = 4000;
const uint32_t REPETE_MEME_MS = 8000;
const int CANAL_VOIX = 0, CANAL_BIP = 1;

const char* NOM_NIVEAU[3] = {"LOIN", "PROCHE", "TRES PROCHE"};
const uint16_t COULEUR_NIVEAU[3] = {TFT_GREEN, TFT_ORANGE, TFT_RED};

// ============================================================ BLUETOOTH (même code que Tab5_Collecte)
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
    if (d[0] != 0x4C || d[1] != 0x00 || d[2] != 0x02 || d[3] != 0x15) return;
    if (memcmp(d + 4, UUID_TABGEO, 16) != 0) return;
    for (int i = 0; i < NB_BALISES; i++) {
      if (TYPE[i] == d[21] && NUM[i] == d[23]) {
        portENTER_CRITICAL(&verrou);
        if (nb_file[i] < TAILLE_FILE) file_rssi[i][nb_file[i]++] = dev.getRSSI();
        vu_ms[i] = millis();
        portEXIT_CRITICAL(&verrou);
      }
    }
  }
};

// ============================================================ FILTRAGE DU RSSI (même code que Tab5_Collecte)
const int N_MED = 5;
float brut[NB_BALISES][N_MED];
int nb_brut[NB_BALISES], idx_brut[NB_BALISES];
float lisse[NB_BALISES];

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
  brut[i][idx_brut[i]] = r;
  idx_brut[i] = (idx_brut[i] + 1) % N_MED;
  if (nb_brut[i] < N_MED) nb_brut[i]++;
  float med = mediane(brut[i], nb_brut[i]);
  lisse[i] = isnan(lisse[i]) ? med : 0.7f * lisse[i] + 0.3f * med;
}

void lire_balises() {
  for (int i = 0; i < NB_BALISES; i++) {
    int16_t copie[TAILLE_FILE];
    uint8_t n;
    portENTER_CRITICAL(&verrou);
    n = nb_file[i];
    memcpy(copie, file_rssi[i], n * sizeof(int16_t));
    nb_file[i] = 0;
    portEXIT_CRITICAL(&verrou);
    for (int k = 0; k < n; k++) ajouter_mesure(i, copie[k]);
  }
}

bool detectee(int i) {
  uint32_t vu;
  portENTER_CRITICAL(&verrou);
  vu = vu_ms[i];
  portEXIT_CRITICAL(&verrou);
  bool ok = vu && (millis() - vu < FENETRE_DETECTION_MS) && !isnan(lisse[i]);
  if (!ok && vu && millis() - vu > 3000) { nb_brut[i] = idx_brut[i] = 0; lisse[i] = NAN; }
  return ok;
}

// ============================================================ HEURE ET TEMPÉRATURE (même code que Tab5_Collecte)
void regler_rtc_si_besoin() {
  if (!M5.Rtc.isEnabled()) return;
  auto dt = M5.Rtc.getDateTime();
  if (dt.date.year >= 2025) return;
  const char* mois = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char m[4] = {__DATE__[0], __DATE__[1], __DATE__[2], 0};
  m5::rtc_datetime_t t;
  t.date.year = atoi(__DATE__ + 7);
  t.date.month = (strstr(mois, m) - mois) / 3 + 1;
  t.date.date = atoi(__DATE__ + 4);
  t.time.hours = atoi(__TIME__);
  t.time.minutes = atoi(__TIME__ + 3);
  t.time.seconds = atoi(__TIME__ + 6);
  M5.Rtc.setDateTime(t);
}

float heure_decimale() {
  if (M5.Rtc.isEnabled()) {
    auto t = M5.Rtc.getTime();
    return t.hours + t.minutes / 60.0f + t.seconds / 3600.0f;
  }
  return fmodf(atoi(__TIME__) + atoi(__TIME__ + 3) / 60.0f + millis() / 3600000.0f, 24.0f);
}

float temperature() {
  float t = NAN;
  if (M5.Imu.isEnabled()) M5.Imu.getTemp(&t);
  return t;
}

// ============================================================ RÉSEAU DE NEURONES
// Même calcul que scikit-learn : normalisation, puis pour chaque couche  sortie = activation(entrée x W + B),
// et à la fin "softmax" pour obtenir 3 probabilités dont la somme vaut 1.
const int MAX_NEURONES = 64;

void predire(const float* x, float* proba) {
  float a[MAX_NEURONES], b[MAX_NEURONES];
  for (int i = 0; i < N_ENTREES; i++) a[i] = (x[i] - MOYENNE[i]) / ECART[i];
  for (int c = 0; c < N_COUCHES; c++) {
    int n_in = TAILLES[c], n_out = TAILLES[c + 1];
    for (int j = 0; j < n_out; j++) {
      float s = BIAIS[c][j];
      for (int i = 0; i < n_in; i++) s += a[i] * POIDS[c][i * n_out + j];
      if (c < N_COUCHES - 1) s = ACTIVATION_RELU ? max(0.0f, s) : tanhf(s);   // couches cachées
      b[j] = s;
    }
    for (int j = 0; j < n_out; j++) a[j] = b[j];
  }
  int n = TAILLES[N_COUCHES];               // softmax
  float m = a[0];
  for (int j = 1; j < n; j++) m = max(m, a[j]);
  float somme = 0;
  for (int j = 0; j < n; j++) { proba[j] = expf(a[j] - m); somme += proba[j]; }
  for (int j = 0; j < n; j++) proba[j] /= somme;
}

// construit les entrées dans le MÊME ordre que dans analyse_reseau.py
int construire_entrees(float* x, const float* det, const float* rssi) {
  int n = 0;
  x[n++] = det[0]; x[n++] = rssi[0]; x[n++] = det[1]; x[n++] = rssi[1];
  if (UTILISE_HEURE) {
    float h = heure_decimale();
    x[n++] = sinf(2 * PI * h / 24); x[n++] = cosf(2 * PI * h / 24);
  }
  if (UTILISE_TEMPERATURE) {
    float t = temperature();
    x[n] = isnan(t) ? MOYENNE[n] : t;   // si pas de capteur : valeur moyenne (neutre)
    n++;
  }
  return n;
}

// ============================================================ SON
bool son = true;
uint32_t dernier_bip = 0, derniere_voix = 0, derniere_meme = 0;
int phrase_precedente = -1;
const uint8_t* VOIX[3] = {nullptr, voix_proche, voix_danger};
const size_t TAILLE_VOIX[3] = {0, sizeof(voix_proche), sizeof(voix_danger)};

bool voix_en_cours() { return M5.Speaker.isPlaying(CANAL_VOIX); }

void jouer(const uint8_t* wav, size_t taille, int id, bool forcer) {
  if (!son || voix_en_cours()) return;
  uint32_t t = millis();
  if (!forcer && t - derniere_voix < PAUSE_VOIX_MS) return;
  if (!forcer && id == phrase_precedente && t - derniere_meme < REPETE_MEME_MS) return;
  M5.Speaker.playWav(wav, taille, 1, CANAL_VOIX, true);
  derniere_voix = t;
  if (id != phrase_precedente || t - derniere_meme >= REPETE_MEME_MS) derniere_meme = t;
  phrase_precedente = id;
}

void bip_niveau(int niveau) {
  if (!son || niveau == 0 || voix_en_cours()) return;
  uint32_t intervalle = (niveau == 2) ? 180 : 700;
  if (millis() - dernier_bip < intervalle) return;
  dernier_bip = millis();
  M5.Speaker.setChannelVolume(CANAL_BIP, niveau == 2 ? 255 : 150);
  M5.Speaker.tone(niveau == 2 ? 1800 : 900, 70, CANAL_BIP);
}

// ============================================================ ÉCRAN
M5Canvas ecran(&M5.Display);
int W, H;
float det_v[NB_BALISES], rssi_v[NB_BALISES], proba_moy[3] = {1, 0, 0};
int niveau = 0;
bool perdu = false;

void dessiner() {
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(top_center);
  ecran.setTextSize(1);
  ecran.setFont(&fonts::FreeSansBold12pt7b);
  ecran.setTextColor(TFT_DARKGREY);
  ecran.drawString("TAB-GEOLOC  -  RESEAU DE NEURONES", W / 2, 30);

  // décision
  uint16_t c = perdu ? TFT_DARKGREY : COULEUR_NIVEAU[niveau];
  ecran.fillRoundRect(40, 100, W - 80, 260, 24, c);
  ecran.setFont(&fonts::FreeSansBold24pt7b);
  ecran.setTextSize(1.5f);
  ecran.setTextColor(TFT_BLACK);
  ecran.setTextDatum(middle_center);
  ecran.drawString(perdu ? "SIGNAL PERDU" : NOM_NIVEAU[niveau], W / 2, 230);

  // probabilités données par le réseau
  ecran.setTextSize(1.0f);
  ecran.setFont(&fonts::FreeSans18pt7b);
  ecran.setTextDatum(top_left);
  ecran.setTextColor(TFT_WHITE);
  ecran.drawString("Probabilites du reseau :", 40, 410);
  for (int k = 0; k < 3; k++) {
    int y = 470 + k * 80;
    ecran.setTextColor(TFT_LIGHTGREY);
    ecran.drawString(NOM_NIVEAU[k], 40, y + 8);
    ecran.drawRect(270, y, W - 310, 50, TFT_DARKGREY);
    ecran.fillRect(273, y + 3, (int)((W - 316) * proba_moy[k]), 44, COULEUR_NIVEAU[k]);
    char p[12]; snprintf(p, sizeof(p), "%3.0f %%", 100 * proba_moy[k]);
    ecran.setTextColor(TFT_WHITE);
    ecran.drawString(p, W - 160, y + 8);
  }

  // entrées
  ecran.setTextColor(TFT_LIGHTGREY);
  char l[80];
  snprintf(l, sizeof(l), "ZONE      det=%d  %6.1f dBm", (int)det_v[0], rssi_v[0]);
  ecran.drawString(l, 40, 760);
  snprintf(l, sizeof(l), "OBSTACLE  det=%d  %6.1f dBm", (int)det_v[1], rssi_v[1]);
  ecran.drawString(l, 40, 815);
  snprintf(l, sizeof(l), "Heure %.2f h   Temp. %.1f C", heure_decimale(), temperature());
  ecran.drawString(l, 40, 870);

  // description du réseau chargé
  char r[80]; int pos = 0;
  pos += snprintf(r + pos, sizeof(r) - pos, "Reseau : ");
  for (int i = 0; i <= N_COUCHES; i++) pos += snprintf(r + pos, sizeof(r) - pos, i ? "-%d" : "%d", TAILLES[i]);
  snprintf(r + pos, sizeof(r) - pos, "  (%s)", ACTIVATION_RELU ? "relu" : "tanh");
  ecran.setTextColor(TFT_DARKGREY);
  ecran.drawString(r, 40, 960);
  ecran.setTextDatum(top_center);
  ecran.drawString(son ? "SON : OUI  (toucher l'ecran pour couper)" : "SON : COUPE  (toucher l'ecran)", W / 2, H - 70);
  ecran.pushSprite(0, 0);
}

// ============================================================ SETUP / LOOP
BLEScan* scan;
float historique[N_VOTE][3];
int n_hist = 0, i_hist = 0;
uint32_t derniere_balise = 0;
bool deja_perdu = false;

void setup() {
  M5.begin();
  Serial.begin(115200);
  if (M5.Display.width() > M5.Display.height()) M5.Display.setRotation(M5.Display.getRotation() ^ 1);
  W = M5.Display.width();
  H = M5.Display.height();
  ecran.setColorDepth(16);
  ecran.setPsram(true);
  ecran.createSprite(W, H);
  for (int i = 0; i < NB_BALISES; i++) { lisse[i] = NAN; nb_brut[i] = idx_brut[i] = 0; nb_file[i] = 0; vu_ms[i] = 0; }
  regler_rtc_si_besoin();
  M5.Speaker.setVolume(220);
  M5.Speaker.setChannelVolume(CANAL_VOIX, 255);

  WiFi.setPins(12, 13, 11, 10, 9, 8, 15);
  BLEDevice::init("");
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new EcouteBalises(), true);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(99);
  scan->start(0, nullptr, false);
  Serial.println("temps_ms;det1;rssi1;det2;rssi2;p_loin;p_proche;p_tres_proche;niveau");
}

uint32_t dernier = 0, dernier_dessin = 0, dernier_controle = 0;

void loop() {
  M5.update();
  lire_balises();

  if (millis() - dernier >= PERIODE_MS) {
    dernier = millis();
    for (int i = 0; i < NB_BALISES; i++) {
      bool d = detectee(i);
      det_v[i] = d ? 1 : 0;
      rssi_v[i] = d ? lisse[i] : RSSI_ABSENT;
    }
    // 1) réseau de neurones
    float x[16], p[3];
    construire_entrees(x, det_v, rssi_v);
    predire(x, p);
    // 2) moyenne sur les 5 dernières décisions
    for (int k = 0; k < 3; k++) historique[i_hist][k] = p[k];
    i_hist = (i_hist + 1) % N_VOTE;
    if (n_hist < N_VOTE) n_hist++;
    for (int k = 0; k < 3; k++) {
      proba_moy[k] = 0;
      for (int h = 0; h < n_hist; h++) proba_moy[k] += historique[h][k];
      proba_moy[k] /= n_hist;
    }
    int ancien = niveau;
    niveau = 0;
    for (int k = 1; k < 3; k++) if (proba_moy[k] > proba_moy[niveau]) niveau = k;

    // 3) balises perdues ?
    if (det_v[0] > 0 || det_v[1] > 0) { derniere_balise = millis(); deja_perdu = false; }
    perdu = derniere_balise && (millis() - derniere_balise > PERTE_MS);
    if (perdu) niveau = 0;

    // 4) voix : quand le danger augmente, puis rappel régulier tant qu'il dure
    if (perdu && !deja_perdu) { deja_perdu = true; jouer(voix_perdu, sizeof(voix_perdu), 3, true); }
    else if (!perdu && niveau > 0) jouer(VOIX[niveau], TAILLE_VOIX[niveau], niveau, false);
    if (niveau != ancien) Serial.printf("# niveau : %s -> %s\n", NOM_NIVEAU[ancien], NOM_NIVEAU[niveau]);

    Serial.printf("%lu;%d;%.1f;%d;%.1f;%.3f;%.3f;%.3f;%s\n", (unsigned long)millis(), (int)det_v[0], rssi_v[0],
                  (int)det_v[1], rssi_v[1], proba_moy[0], proba_moy[1], proba_moy[2], NOM_NIVEAU[niveau]);
  }

  bip_niveau(perdu ? 0 : niveau);   // le bip indique le niveau en continu

  if (M5.Touch.getCount() > 0 && M5.Touch.getDetail().wasPressed()) {
    son = !son;
    if (!son) M5.Speaker.stop();
    M5.Speaker.setChannelVolume(CANAL_BIP, 150);
    M5.Speaker.tone(son ? 1500 : 500, 80, CANAL_BIP);
  }

  if (millis() - dernier_dessin > 200) { dernier_dessin = millis(); dessiner(); }

  if (millis() - dernier_controle >= 5000) {
    dernier_controle = millis();
    if (!scan->isScanning()) scan->start(0, nullptr, false);
    scan->clearResults();
  }
}
