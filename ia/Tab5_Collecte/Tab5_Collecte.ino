/*
  TAB-Géoloc – Tab5 : COLLECTE DES MESURES pour entraîner le réseau de neurones
  -----------------------------------------------------------------------------
  Carte        : "M5Tab5" (paquet M5Stack 3.3.x ou "esp32 by Espressif" 3.3.1 ou plus)
  Bibliothèques: M5Unified >= 0.2.23, M5GFX >= 0.2.30

  Principe (environnement connu = "ad hoc") :
    1. On marque au sol des distances par rapport à la balise OBSTACLE (0,5 m, 1 m, 1,5 m ... 8 m).
    2. On se place sur une marque avec le Tab5 porté sur le torse, on touche le bouton de cette distance.
    3. Le Tab5 enregistre pendant 15 s (5 mesures par seconde) :
         det1, rssi1 (balise ZONE), det2, rssi2 (balise OBSTACLE), heure, température
    4. Les mesures partent sur le port USB : sur le PC, lancer
         python analyse_reseau.py capture COM5 mesures/salle.csv
  Conseils : au moins 3 enregistrements par distance, en changeant un peu d'orientation,
             à des moments différents, avec des gens qui passent ou non.
*/
#include <M5Unified.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ============================================================ RÉGLAGES (identiques à Tab5_IA)
const uint8_t UUID_TABGEO[16] = {0x7A, 0xB0, 0x6E, 0x01, 0x5E, 0xE5, 0x40, 0x26,
                                 0x9A, 0x1B, 0x15, 0x7E, 0x5E, 0xE5, 0x20, 0x26};
const int NB_BALISES = 2;
const uint8_t TYPE[NB_BALISES] = {1, 2};          // balise 1 = ZONE, balise 2 = OBSTACLE
const uint8_t NUM[NB_BALISES]  = {1, 1};
const uint32_t FENETRE_DETECTION_MS = 1000;       // "détectée" = au moins une trame dans la dernière seconde
const float RSSI_ABSENT = -100.0f;                // valeur mise quand la balise n'est pas détectée
const uint32_t PERIODE_MS = 200;                  // 5 mesures par seconde
const uint32_t DUREE_ENREG_MS = 15000;            // 15 s par enregistrement

const int NB_DIST = 9;
const float DISTANCES[NB_DIST] = {0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f, 99.0f};   // 99 = hors de portée
const char* ETIQ[NB_DIST] = {"0,5 m", "1 m", "1,5 m", "2 m", "3 m", "4 m", "6 m", "8 m", "HORS"};

// ============================================================ BLUETOOTH (même code que Tab5_IA)
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

// ============================================================ FILTRAGE DU RSSI (même code que Tab5_IA)
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
  if (!ok && vu && millis() - vu > 3000) { nb_brut[i] = idx_brut[i] = 0; lisse[i] = NAN; }  // balise perdue : on repart de zéro
  return ok;
}

// ============================================================ HEURE ET TEMPÉRATURE
void regler_rtc_si_besoin() {
  if (!M5.Rtc.isEnabled()) return;
  auto dt = M5.Rtc.getDateTime();
  if (dt.date.year >= 2025) return;            // déjà à l'heure
  // sinon : on prend l'heure de la compilation du programme
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

float temperature() {           // capteur interne du Tab5 (dans la centrale inertielle) : indicatif
  float t = NAN;
  if (M5.Imu.isEnabled()) M5.Imu.getTemp(&t);
  return t;
}

// ============================================================ ÉCRAN
M5Canvas ecran(&M5.Display);
int W, H;
const int GX = 30, GY = 600, BW = 210, BH = 170, ESP = 15;   // grille des boutons
bool enreg = false;
int dist_choisie = -1;
uint32_t debut_enreg = 0, session = 0, nb_lignes = 0, nb_enreg = 0;
float det_aff[NB_BALISES], rssi_aff[NB_BALISES];

void dessiner() {
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(top_center);
  ecran.setTextSize(1);
  ecran.setFont(&fonts::FreeSansBold12pt7b);
  ecran.setTextColor(TFT_DARKGREY);
  ecran.drawString("TAB-GEOLOC  -  COLLECTE DES MESURES", W / 2, 30);

  ecran.setFont(&fonts::FreeSans18pt7b);
  ecran.setTextDatum(top_left);
  const char* noms[NB_BALISES] = {"ZONE", "OBSTACLE"};
  for (int i = 0; i < NB_BALISES; i++) {
    char l[64];
    if (det_aff[i] > 0) snprintf(l, sizeof(l), "%-9s det = 1   %5.1f dBm", noms[i], rssi_aff[i]);
    else snprintf(l, sizeof(l), "%-9s det = 0   (absente)", noms[i]);
    ecran.setTextColor(det_aff[i] > 0 ? TFT_GREEN : TFT_ORANGE);
    ecran.drawString(l, 40, 100 + i * 60);
  }
  char l2[64];
  float tp = temperature();
  snprintf(l2, sizeof(l2), "Heure %.2f h    Temp. %.1f C", heure_decimale(), tp);
  ecran.setTextColor(TFT_LIGHTGREY);
  ecran.drawString(l2, 40, 230);

  ecran.setTextDatum(top_center);
  if (enreg) {
    uint32_t reste = (DUREE_ENREG_MS - (millis() - debut_enreg)) / 1000 + 1;
    ecran.setFont(&fonts::FreeSansBold24pt7b);
    ecran.setTextSize(1.5f);
    ecran.setTextColor(TFT_RED);
    char t[48];
    snprintf(t, sizeof(t), "ENREG. %s", ETIQ[dist_choisie]);
    ecran.drawString(t, W / 2, 360);
    snprintf(t, sizeof(t), "%lu s", (unsigned long)reste);
    ecran.setTextSize(3.0f);
    ecran.setTextColor(TFT_WHITE);
    ecran.drawString(t, W / 2, 560);
    ecran.setTextSize(1.0f);
    ecran.setFont(&fonts::FreeSans18pt7b);
    ecran.drawString("Ne bougez pas de la marque au sol", W / 2, 900);
  } else {
    ecran.setFont(&fonts::FreeSans18pt7b);
    ecran.setTextColor(TFT_WHITE);
    ecran.drawString("Distance a la balise OBSTACLE :", W / 2, 400);
    char t[64];
    snprintf(t, sizeof(t), "%lu enregistrements faits", (unsigned long)nb_enreg);
    ecran.setTextColor(TFT_DARKGREY);
    ecran.drawString(t, W / 2, 470);
    ecran.setFont(&fonts::FreeSansBold24pt7b);
    for (int k = 0; k < NB_DIST; k++) {
      int x = GX + (k % 3) * (BW + ESP), y = GY + (k / 3) * (BH + ESP);
      ecran.fillRoundRect(x, y, BW, BH, 18, k == NB_DIST - 1 ? 0x4208 : 0x1A69);
      ecran.setTextColor(TFT_WHITE);
      ecran.setTextDatum(middle_center);
      ecran.drawString(ETIQ[k], x + BW / 2, y + BH / 2);
    }
  }
  ecran.pushSprite(0, 0);
}

int bouton_touche(int tx, int ty) {
  for (int k = 0; k < NB_DIST; k++) {
    int x = GX + (k % 3) * (BW + ESP), y = GY + (k / 3) * (BH + ESP);
    if (tx >= x && tx < x + BW && ty >= y && ty < y + BH) return k;
  }
  return -1;
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
  for (int i = 0; i < NB_BALISES; i++) { lisse[i] = NAN; nb_brut[i] = idx_brut[i] = 0; nb_file[i] = 0; vu_ms[i] = 0; }
  regler_rtc_si_besoin();
  M5.Speaker.setVolume(180);
  session = esp_random() % 900000 + 100000;   // numéro unique (évite les doublons si le Tab5 redémarre)

  WiFi.setPins(12, 13, 11, 10, 9, 8, 15);
  BLEDevice::init("");
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new EcouteBalises(), true);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(99);
  scan->start(0, nullptr, false);
  Serial.println("TAB-Geoloc collecte prete");
}

uint32_t dernier = 0, dernier_dessin = 0, dernier_controle = 0;

void loop() {
  M5.update();
  lire_balises();

  // toucher un bouton = démarrer un enregistrement de 15 s
  if (!enreg && M5.Touch.getCount() > 0) {
    auto t = M5.Touch.getDetail();
    if (t.wasPressed()) {
      int k = bouton_touche(t.x, t.y);
      if (k >= 0) {
        dist_choisie = k; enreg = true; debut_enreg = millis(); session++; nb_lignes = 0;
        M5.Speaker.tone(1500, 150);
        Serial.printf("# debut session %lu distance %.1f m\n", (unsigned long)session, DISTANCES[k]);
      }
    }
  }

  // une mesure toutes les 200 ms
  if (millis() - dernier >= PERIODE_MS) {
    dernier = millis();
    for (int i = 0; i < NB_BALISES; i++) {
      bool d = detectee(i);
      det_aff[i] = d ? 1 : 0;
      rssi_aff[i] = d ? lisse[i] : RSSI_ABSENT;
    }
    if (enreg) {
      Serial.printf("M;%lu;%lu;%.1f;%d;%.1f;%d;%.1f;%.4f;%.1f\n", (unsigned long)session,
                    (unsigned long)(millis() - debut_enreg), DISTANCES[dist_choisie],
                    (int)det_aff[0], rssi_aff[0], (int)det_aff[1], rssi_aff[1], heure_decimale(), temperature());
      nb_lignes++;
      if (millis() - debut_enreg >= DUREE_ENREG_MS) {
        enreg = false; nb_enreg++;
        M5.Speaker.tone(800, 120); delay(160); M5.Speaker.tone(800, 120);
        Serial.printf("# fin session %lu : %lu mesures\n", (unsigned long)session, (unsigned long)nb_lignes);
      }
    }
  }

  if (millis() - dernier_dessin > 250) { dernier_dessin = millis(); dessiner(); }

  if (millis() - dernier_controle >= 5000) {
    dernier_controle = millis();
    if (!scan->isScanning()) scan->start(0, nullptr, false);
    scan->clearResults();
  }
}
