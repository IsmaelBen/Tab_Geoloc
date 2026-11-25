/*
 * TAB-Géoloc – Test de communication sans fil Wi-Fi entre 2 M5Stack Tab5
 * -----------------------------------------------------------------------
 * Même programme pour les deux Tab5 : changer seulement ROLE avant de téléverser.
 *
 *   ROLE 1 = Tab5 FIXE   : crée le réseau Wi-Fi et répond aux messages (« PONG »)
 *   ROLE 2 = Tab5 MOBILE : se connecte, envoie un message toutes les 200 ms (« PING »)
 *                          et affiche : RSSI (dBm), paquets envoyés / reçus, perte (%), temps aller-retour (ms)
 *
 * Arduino IDE : carte « M5Tab5 » (paquet M5Stack >= 3.2.2),
 *               bibliothèques M5Unified >= 0.2.23 et M5GFX >= 0.2.30.
 * Moniteur série à 115200 bauds : le Tab5 mobile écrit une ligne CSV par seconde (pour le rapport de test).
 * Toucher l'écran du Tab5 mobile remet les compteurs à zéro (à faire à chaque nouvelle distance).
 */

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ======================= À CHANGER AVANT DE TÉLÉVERSER =======================
#define ROLE 1            // 1 = Tab5 fixe (point d'accès), 2 = Tab5 mobile

// 0 = le Tab5 fixe crée lui-même le réseau Wi-Fi (rien d'autre à prévoir)
// 1 = les deux Tab5 se connectent au partage de connexion d'un téléphone
#define MODE_PARTAGE 0
const char* PARTAGE_SSID = "NomDuPartage";
const char* PARTAGE_MDP  = "MotDePasse";
// =============================================================================

const char* AP_SSID = "TAB-GEOLOC-TEST";   // réseau créé par le Tab5 fixe
const char* AP_MDP  = "tabgeoloc2026";     // 8 caractères minimum
const uint16_t PORT = 4210;                // port UDP utilisé par les deux Tab5
const uint32_t PERIODE_PING_MS = 200;      // un message toutes les 200 ms

// Liaison SDIO entre l'ESP32-P4 et la puce Wi-Fi ESP32-C6 du Tab5 (doc M5Stack)
const int SDIO_CLK = 12, SDIO_CMD = 13, SDIO_D0 = 11, SDIO_D1 = 10,
          SDIO_D2 = 9, SDIO_D3 = 8, SDIO_RST = 15;

WiFiUDP udp;
char buffer[64];

// Compteurs (Tab5 mobile)
uint32_t numero = 0, envoyes = 0, recus = 0;
uint32_t somme_rtt = 0, rtt_max = 0;
uint32_t dernier_ping = 0, dernier_affichage = 0, dernier_csv = 0;

// Compteurs (Tab5 fixe)
uint32_t recus_fixe = 0;
IPAddress dernier_emetteur;

// ----------------------------------------------------------------------------- affichage
void ligne(int n, const char* texte, uint16_t couleur = TFT_WHITE) {
  M5.Display.setTextColor(couleur, TFT_BLACK);
  M5.Display.setCursor(20, 40 + n * 60);
  M5.Display.printf("%-30s", texte);       // espaces en plus pour effacer l'ancien texte
}

void titre(const char* t) {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(3);
  ligne(0, t, TFT_CYAN);
}

// ----------------------------------------------------------------------------- Wi-Fi
void demarrer_wifi() {
  WiFi.setPins(SDIO_CLK, SDIO_CMD, SDIO_D0, SDIO_D1, SDIO_D2, SDIO_D3, SDIO_RST);

#if ROLE == 1 && MODE_PARTAGE == 0
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_MDP);
  ligne(1, "Reseau cree :");
  ligne(2, AP_SSID, TFT_YELLOW);
  snprintf(buffer, sizeof(buffer), "IP : %s", WiFi.softAPIP().toString().c_str());
  ligne(3, buffer);
#else
  const char* ssid = MODE_PARTAGE ? PARTAGE_SSID : AP_SSID;
  const char* mdp  = MODE_PARTAGE ? PARTAGE_MDP  : AP_MDP;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, mdp);
  ligne(1, "Connexion a :");
  ligne(2, ssid, TFT_YELLOW);
  int points = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    snprintf(buffer, sizeof(buffer), "Attente%.*s", (points++ % 4), "...");
    ligne(3, buffer);
  }
  snprintf(buffer, sizeof(buffer), "IP : %s", WiFi.localIP().toString().c_str());
  ligne(3, buffer, TFT_GREEN);
#endif
  udp.begin(PORT);
}

// Adresse du Tab5 fixe vue par le Tab5 mobile
IPAddress adresse_fixe() {
#if MODE_PARTAGE == 0
  return IPAddress(192, 168, 4, 1);          // adresse par défaut du point d'accès
#else
  return IPAddress(255, 255, 255, 255);      // diffusion : le Tab5 fixe répond directement
#endif
}

// ----------------------------------------------------------------------------- setup
void setup() {
  M5.begin();
  Serial.begin(115200);
#if ROLE == 1
  titre("TAB-Geoloc : Tab5 FIXE");
#else
  titre("TAB-Geoloc : Tab5 MOBILE");
#endif
  demarrer_wifi();
#if ROLE == 2
  Serial.println("temps_ms;rssi_dBm;envoyes;recus;perte_pct;rtt_moy_ms;rtt_max_ms");
#endif
}

// ----------------------------------------------------------------------------- Tab5 FIXE
void boucle_fixe() {
  int taille = udp.parsePacket();
  if (taille > 0) {
    int n = udp.read(buffer, sizeof(buffer) - 1);
    buffer[n] = '\0';
    if (strncmp(buffer, "PING;", 5) == 0) {
      buffer[1] = 'O';                         // « PING;... » devient « PONG;... » (même numéro, même heure)
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write((const uint8_t*)buffer, n);
      udp.endPacket();
      recus_fixe++;
      dernier_emetteur = udp.remoteIP();
    }
  }
  if (millis() - dernier_affichage > 500) {
    dernier_affichage = millis();
    snprintf(buffer, sizeof(buffer), "Messages recus : %lu", (unsigned long)recus_fixe);
    ligne(5, buffer);
    snprintf(buffer, sizeof(buffer), "De : %s", dernier_emetteur.toString().c_str());
    ligne(6, buffer);
#if MODE_PARTAGE == 1
    snprintf(buffer, sizeof(buffer), "RSSI : %d dBm", WiFi.RSSI());
    ligne(7, buffer);
#endif
  }
}

// ----------------------------------------------------------------------------- Tab5 MOBILE
void boucle_mobile() {
  // 1) envoyer un PING toutes les 200 ms
  if (millis() - dernier_ping >= PERIODE_PING_MS) {
    dernier_ping = millis();
    int n = snprintf(buffer, sizeof(buffer), "PING;%lu;%lu", (unsigned long)numero++, (unsigned long)millis());
    udp.beginPacket(adresse_fixe(), PORT);
    udp.write((const uint8_t*)buffer, n);
    udp.endPacket();
    envoyes++;
  }

  // 2) lire les réponses PONG et calculer le temps aller-retour
  int taille = udp.parsePacket();
  if (taille > 0) {
    int n = udp.read(buffer, sizeof(buffer) - 1);
    buffer[n] = '\0';
    unsigned long num, t_envoi;
    if (sscanf(buffer, "PONG;%lu;%lu", &num, &t_envoi) == 2) {
      uint32_t rtt = millis() - t_envoi;
      recus++;
      somme_rtt += rtt;
      if (rtt > rtt_max) rtt_max = rtt;
    }
  }

  // 3) toucher l'écran = remise à zéro (nouvelle distance de mesure)
  if (M5.Touch.getCount() > 0 && M5.Touch.getDetail().wasPressed()) {
    envoyes = recus = somme_rtt = rtt_max = 0;
    M5.Speaker.tone(2000, 100);
  }

  int rssi = WiFi.RSSI();
  float perte = envoyes ? 100.0f * (envoyes - recus) / envoyes : 0;
  uint32_t rtt_moy = recus ? somme_rtt / recus : 0;

  // 4) affichage toutes les 500 ms
  if (millis() - dernier_affichage > 500) {
    dernier_affichage = millis();
    snprintf(buffer, sizeof(buffer), "RSSI : %d dBm", rssi);
    ligne(5, buffer, rssi > -60 ? TFT_GREEN : (rssi > -75 ? TFT_YELLOW : TFT_RED));
    snprintf(buffer, sizeof(buffer), "Envoyes : %lu", (unsigned long)envoyes);
    ligne(6, buffer);
    snprintf(buffer, sizeof(buffer), "Recus   : %lu", (unsigned long)recus);
    ligne(7, buffer);
    snprintf(buffer, sizeof(buffer), "Perte   : %.1f %%", perte);
    ligne(8, buffer, perte < 5 ? TFT_GREEN : TFT_RED);
    snprintf(buffer, sizeof(buffer), "Aller-retour : %lu ms", (unsigned long)rtt_moy);
    ligne(9, buffer);
    ligne(11, "Toucher l'ecran = remise a zero", TFT_DARKGREY);
    if (WiFi.status() != WL_CONNECTED) ligne(10, "Wi-Fi perdu !", TFT_RED);
    else ligne(10, "");
  }

  // 5) une ligne CSV par seconde sur le port série (pour le rapport)
  if (millis() - dernier_csv >= 1000) {
    dernier_csv = millis();
    Serial.printf("%lu;%d;%lu;%lu;%.1f;%lu;%lu\n", (unsigned long)millis(), rssi, (unsigned long)envoyes,
                  (unsigned long)recus, perte, (unsigned long)rtt_moy, (unsigned long)rtt_max);
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
