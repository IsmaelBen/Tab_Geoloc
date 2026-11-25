/*
  >>> STEFFY – balise OBSTACLE 1 – carte ESP-32U (USB-C). Brancher l'antenne AVANT. Rien à modifier : téléverser tel quel.
  TAB-Géoloc – Programme des BALISES BLE (trame iBeacon)
  ------------------------------------------------------
  Carte      : "ESP32 Dev Module" (paquet "esp32 by Espressif" version 3.3.1 ou plus)
  Rôle       : émet une trame iBeacon toutes les 100 ms à +3 dBm (valeurs de la spec)
  Avant de téléverser : régler TYPE_BALISE et NUMERO_BALISE ci-dessous.

  Nos 2 balises :
    - ESP-32  (prise micro-USB, antenne imprimée) -> TYPE_BALISE 1 (ZONE),     NUMERO_BALISE 1
    - ESP-32U (prise USB-C, antenne externe)      -> TYPE_BALISE 2 (OBSTACLE), NUMERO_BALISE 1
  ATTENTION : sur l'ESP-32U, brancher l'antenne AVANT d'alimenter la carte.
*/
#include <BLEDevice.h>
#include <BLEAdvertising.h>

#define TYPE_BALISE   2   // 1 = ZONE, 2 = OBSTACLE  (champ "major" de la trame)
#define NUMERO_BALISE 1   // numéro de la balise      (champ "minor" de la trame)

// Identifiant commun à toutes nos balises : le Tab5 ignore les autres appareils Bluetooth.
// (doit être IDENTIQUE dans le programme du Tab5)
const uint8_t UUID_TABGEO[16] = {0x7A, 0xB0, 0x6E, 0x01, 0x5E, 0xE5, 0x40, 0x26,
                                 0x9A, 0x1B, 0x15, 0x7E, 0x5E, 0xE5, 0x20, 0x26};
const int8_t PUISSANCE_1M = -59;  // champ "puissance à 1 m" de la trame (indicatif, non utilisé par le Tab5)
const int LED = 2;                // LED bleue présente sur la plupart des cartes ESP32

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  BLEDevice::init("TABGEO");
  BLEDevice::setPower(ESP_PWR_LVL_P3);  // +3 dBm (choix de la spec)

  // Trame iBeacon (25 octets) : 4C 00 | 02 15 | UUID (16) | major (2) | minor (2) | puissance (1)
  uint8_t t[25] = {0x4C, 0x00, 0x02, 0x15};
  memcpy(t + 4, UUID_TABGEO, 16);
  t[20] = 0;  t[21] = TYPE_BALISE;
  t[22] = 0;  t[23] = NUMERO_BALISE;
  t[24] = (uint8_t)PUISSANCE_1M;
  String trame;
  for (int i = 0; i < 25; i++) trame += (char)t[i];

  BLEAdvertisementData donnees;
  donnees.setFlags(0x04);  // Bluetooth LE seul
  donnees.setManufacturerData(trame);

  BLEAdvertising* annonce = BLEDevice::getAdvertising();
  annonce->setAdvertisementData(donnees);
  annonce->setScanResponse(false);
#if defined(CONFIG_BLUEDROID_ENABLED)
  annonce->setAdvertisementType(ADV_TYPE_NONCONN_IND);   // non connectable : on émet seulement
#elif defined(CONFIG_NIMBLE_ENABLED)
  annonce->setAdvertisementType(BLE_GAP_CONN_MODE_NON);
#endif
  annonce->setMinInterval(160);  // 160 x 0,625 ms = 100 ms entre deux trames
  annonce->setMaxInterval(160);
  annonce->start();

  Serial.printf("Balise %s n°%d : 1 trame toutes les 100 ms a +3 dBm\n",
                TYPE_BALISE == 1 ? "ZONE" : "OBSTACLE", NUMERO_BALISE);
}

void loop() {
  digitalWrite(LED, (millis() / 1000) % 2);  // la LED clignote : la balise fonctionne
  delay(50);
}
