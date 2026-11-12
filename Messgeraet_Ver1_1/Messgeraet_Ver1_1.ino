/*
  Messgeraet Ver1 – Firmware fuer die Platine "Ver1" (ESP32-S3-WROOM-1-N16R8)
  ============================================================================
  Basiert auf Messgeraut_V5 (Bluetooth-Prototyp). Die BLE-Schnittstelle zur
  Weboberflaeche (index.html) ist beibehalten und um zwei Characteristics
  erweitert: STATUS (Akku, SD-Karte, Langzeitmessung) und FILES (Dateien auf
  der SD-Karte auflisten, herunterladen, loeschen).

  ---------------------------------------------------------------------------
  Hardware laut Schaltplan Ver1 (KiCad-Projekt "Ver1", Netzliste der Platine)
  ---------------------------------------------------------------------------
    Digitaleingaenge  DI1..DI4 -> IO4, IO5, IO6, IO7
                      ISO1211 (U201..U204), Ausgang HIGH = 24 V liegt an
    I2C               SDA IO8, SCL IO9 (Pull-ups 4k7 auf der Platine)
      MCP3424  0x68   AI1..AI4, differenziell, Eingangsteiler 100k/10k (1:11),
                      Mittenspannung VCM = 1,65 V, Puffer MCP6004
      ADS7828  0x48   CH0..CH2 = Netzspannung L1..L3 gegen N ueber ZMPT101B
                      (4 x 56k primaer, Buerde 510R), CH3 = Mittenspannung,
                      interne Referenz 2,5 V
    SD-Karte          SDMMC 4 Bit: CLK IO39, CMD IO38, D0 IO40, D1 IO41,
                      D2 IO42, D3 IO47, Kartenerkennung IO48 (LOW = Karte steckt)
    Akkuspannung      IO1 (ADC1_CH0), Teiler R10 100k / R11 15k an VBAT_SW
    USB-C             IO19 (D-) / IO20 (D+), nativer USB des ESP32-S3
                      -> Programmieren und serieller Monitor
    Taster            BOOT = IO0, RESET = EN

  Hinweise zur Platine:
    - VBUS der USB-Buchse versorgt die Platine NICHT. Zum Flashen und fuer den
      seriellen Monitor muss der Akku stecken.
    - Die Unterspannungsabschaltung (U2/Q2) trennt den Akku hardwareseitig bei
      ca. 15,1 V und schaltet erst ab ca. 16,1 V wieder zu. Die Firmware
      schliesst die Messdatei vorher sauber (BAT_CRIT_V).

  ---------------------------------------------------------------------------
  Arduino-IDE (ESP32-Boardpaket 3.x)
  ---------------------------------------------------------------------------
    Board             "ESP32S3 Dev Module"
    USB CDC On Boot   "Enabled"            (Serial laeuft dann ueber USB-C)
    USB Mode          "Hardware CDC and JTAG"
    Flash Size        "16MB (128Mb)"
    Partition Scheme  "16M Flash (3MB APP/9.9MB FATFS)"
    PSRAM             "OPI PSRAM"
    Upload Mode       "UART0 / Hardware CDC"

    Bibliothek:  NimBLE-Arduino (h2zero), Version 2.x
                 (ArduinoJson und Adafruit ADS1X15 werden nicht mehr gebraucht)

    Erstes Flashen einer fabrikneuen Platine: BOOT gedrueckt halten, RESET
    kurz druecken, BOOT loslassen, dann hochladen. Danach einmal RESET.

  ---------------------------------------------------------------------------
  SD-Karte / Langzeitmessung
  ---------------------------------------------------------------------------
    - Karte muss FAT32-formatiert sein (exFAT wird vom ESP32-Core nicht
      unterstuetzt, Karten > 32 GB also vorher mit FAT32 formatieren).
    - Dateien: LOG_JJJJMMTT_HHMMSS.csv (Uhrzeit kommt beim Verbinden vom
      Handy/Laptop), ohne bekannte Uhrzeit LOG_0001.csv, LOG_0002.csv, ...
    - Aufbau: Semikolon als Trennzeichen, Dezimalkomma, alle Spannungen in Volt.
      Spalten: Zeit; t_ms; DI 1..4 (0/1); AI 1..4 (V); L1..L3 (V); Marker
      Ein Marker steht als "M<Nr>" in der letzten Spalte der Zeile, die im
      Moment des Setzens geschrieben wird (dafuer wird sofort eine Zeile erzeugt).
    - Eine Zeile je Intervall; zusaetzlich sofort eine Zeile bei jedem
      Wechsel eines Digitaleingangs oder einer Netzspannung (LOG_ON_CHANGE)
      und bei jedem gesetzten Marker.
    - Alle LOG_FLUSH_MS wird auf die Karte gesichert. Wird der Akku gezogen,
      gehen hoechstens die letzten Sekunden verloren.
    - Lief beim Abziehen des Akkus eine Aufzeichnung, wird sie beim naechsten
      Start in einer neuen Datei fortgesetzt (LOG_AUTO_RESUME).

  ---------------------------------------------------------------------------
  Nullabgleich der Analogeingaenge
  ---------------------------------------------------------------------------
    Die Eingangspuffer haben einen Offset von wenigen mV, der mit dem Teiler-
    faktor 11 als 10..50 mV in der Anzeige erscheint. Der Nullabgleich (Knopf
    in der Weboberflaeche unter "Kalibrierung") misst diesen Wert bei
    kurzgeschlossenen Eingaengen (AI+ mit AI- gebrueckt), speichert ihn
    dauerhaft im Geraet und zieht ihn von da an ab. Kanaele, an denen dabei
    mehr als AI_ZERO_MAX_V anliegen, gelten als nicht kurzgeschlossen und
    behalten ihren bisherigen Wert.

  Serieller Monitor (115200 Baud): 's' + Enter gibt den Geraetestatus aus.
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <FS.h>
#include <SD_MMC.h>
#include <Preferences.h>
#include <sys/time.h>
#include <time.h>

// ====================================================================
//  Konfiguration
// ====================================================================
const char* FW_VERSION      = "Ver1-1.3";
const char* BLE_DEVICE_NAME = "Messgeraet";

const uint32_t SAMPLE_INTERVAL_MS = 40;                          // 25 Messwerte pro Sekunde
const uint32_t BUFFER_MINUTES     = 1;                           // Ringpuffer fuer die Live-Ansicht
const uint32_t BUFFER_SIZE = (BUFFER_MINUTES * 60 * 1000UL) / SAMPLE_INTERVAL_MS;
const uint8_t  MAX_HISTORY_BATCH   = 5;                          // 5 * 48 Byte = 240 Byte, passt in MTU 247
const uint8_t  MAX_BLE_CONNECTIONS = 3;                          // z.B. Handy + Laptop gleichzeitig

// ---------- Pinbelegung (Schaltplan Ver1) ----------
const uint8_t PIN_DI[4]    = { 4, 5, 6, 7 };   // DI1..DI4, ISO1211 OUT
const uint8_t PIN_I2C_SDA  = 8;
const uint8_t PIN_I2C_SCL  = 9;
const uint8_t PIN_VBAT_ADC = 1;
const uint8_t PIN_SD_CLK   = 39;
const uint8_t PIN_SD_CMD   = 38;
const uint8_t PIN_SD_D0    = 40;
const uint8_t PIN_SD_D1    = 41;
const uint8_t PIN_SD_D2    = 42;
const uint8_t PIN_SD_D3    = 47;
const uint8_t PIN_SD_CD    = 48;

// ---------- Analogeingaenge AI1..AI4 (MCP3424) ----------
const uint8_t MCP3424_ADDR = 0x68;             // Adr0 = Adr1 = GND
const uint8_t MCP_RES_BITS = 12;               // 12 Bit (240 Wandlungen/s), 14 Bit (60/s) oder 16 Bit (15/s)
                                               // 12 Bit: alle 4 Kanaele in ca. 30 ms, Aufloesung 11 mV am Eingang
const uint8_t MCP_PGA      = 1;                // 1: Messbereich ca. -16..+19 V, 2: +-11,2 V (doppelte Aufloesung)
const float   AI_DIVIDER   = 11.0f;            // (100k + 10k) / 10k
const float   AI_GAIN_CAL[4]  = { 1.0f, 1.0f, 1.0f, 1.0f };   // Feinabgleich Steigung je Kanal
const float   AI_OFFSET_V[4]  = { 0.0f, 0.0f, 0.0f, 0.0f };   // fester Nullpunkt je Kanal (normalerweise 0 lassen
                                                              // und den Nullabgleich der Weboberflaeche benutzen)
const float   AI_ZERO_MAX_V   = 0.25f;         // Nullabgleich: groessere Werte = Eingang nicht kurzgeschlossen
const uint8_t AI_ZERO_SAMPLES = 32;            // Nullabgleich: Messungen je Kanal (ca. 1 s bei 12 Bit)

// ---------- Netzspannung L1..L3 (ADS7828 + ZMPT101B) ----------
const uint8_t  ADS7828_ADDR         = 0x48;    // A0 = A1 = GND
const float    AC_R_PRIMARY         = 4 * 56000.0f;   // R401..R404 (bzw. R405..R408, R409..R412)
const float    AC_R_BURDEN          = 510.0f;         // R413..R415
const float    AC_VREF              = 2.5f;           // interne Referenz des ADS7828
const float    AC_CAL[3]            = { 1.0f, 1.0f, 1.0f };   // Feinabgleich je Phase (Vergleich mit Multimeter)
const uint32_t AC_SAMPLE_PERIOD_MS  = 2;       // 500 Abtastungen/s je Phase = 10 je Netzperiode
const uint32_t AC_WINDOW_MS         = 40;      // Effektivwert ueber 2 Netzperioden (50 Hz)
const float    AC_ZERO_BELOW_V      = 3.0f;    // darunter wird 0 V angezeigt (Rauschen)
const float    AC_EVENT_THRESHOLD_V = 115.0f;  // "Spannung da / weg" fuer die Aenderungserkennung beim Loggen

// ---------- Akku ----------
const float BAT_DIVIDER     = (100.0f + 15.0f) / 15.0f;   // R10 / R11
const float BAT_CAL         = 1.0f;     // Feinabgleich (Vergleich mit Multimeter)
const float BAT_WARN_V      = 16.8f;    // Warnung "Akku schwach"
const float BAT_CRIT_V      = 16.0f;    // Langzeitmessung wird sauber beendet (Hardware trennt bei ca. 15,1 V)
const float BAT_VALID_MIN_V = 8.0f;     // darunter gilt die Messung als "kein Akku" (z.B. Labornetzteil-Test)

// ---------- SD-Karte / Langzeitmessung ----------
const bool     SD_USE_CARD_DETECT   = true;    // false: Kartenerkennung ignorieren, immer versuchen zu mounten
const bool     SD_CD_ACTIVE_LOW     = true;    // DET-Schalter zieht IO48 bei gesteckter Karte auf GND
const uint32_t LOG_DEFAULT_INTERVAL_MS = 1000;
const uint32_t LOG_FLUSH_MS         = 5000;    // so oft wird auf die Karte gesichert
const uint32_t LOG_MAX_ROWS_PER_FILE = 1000000;   // danach neue Datei (Excel-Grenze: 1.048.576 Zeilen)
const uint64_t LOG_MIN_FREE_BYTES   = 8ULL * 1024 * 1024;
const bool     LOG_ON_CHANGE        = true;    // Flanken sofort loggen, unabhaengig vom Intervall
const bool     LOG_AUTO_RESUME      = true;    // nach Akkuwechsel weiterloggen
const char     CSV_SEPARATOR        = ';';     // Semikolon + Dezimalkomma: deutsches Excel oeffnet die Datei direkt richtig.
const bool     CSV_DECIMAL_COMMA    = true;    // (Mit Dezimalpunkt liest Excel z.B. 9.988 V als 9988.)

// ---------- BLE-UUIDs (muessen exakt mit index.html uebereinstimmen) ----------
#define SERVICE_UUID       "79a16fed-1190-4e22-ae20-2b580b72f82f"
#define CHANNELS_CHAR_UUID "ca8eed79-c540-460e-827e-4486c6405eec"
#define SAMPLE_CHAR_UUID   "083cfd49-1af0-4781-ad1b-de854566c8bb"
#define CONTROL_CHAR_UUID  "47a03c81-b4e6-4f47-b8b1-8668beeac41c"
#define HISTORY_CHAR_UUID  "1079034a-acd2-460c-844e-3bcab8d5eb1b"
#define MARKERS_CHAR_UUID  "0a2a3e4c-b842-4454-ad48-062a179e480f"
#define STATUS_CHAR_UUID   "69d5e8df-21e0-4d70-9ddc-b1b9346a82c4"   // neu in Ver1
#define FILES_CHAR_UUID    "f3025d1a-1296-4630-8a0c-bfa2d5802520"   // neu in Ver1

// ====================================================================
//  Kanaele
// ====================================================================
enum ChannelType   : uint8_t { CH_DIGITAL, CH_ANALOG };
enum ChannelSource : uint8_t { SRC_DI, SRC_AI, SRC_AC };

struct Channel {
  const char*   name;
  ChannelType   type;
  float         minVal;
  float         maxVal;
  const char*   unit;       // Einheit fuer die Anzeige
  uint8_t       decimals;   // Nachkommastellen in der Anzeige (CSV: eine mehr)
  ChannelSource src;
  uint8_t       index;      // 0-basierter Index innerhalb der Quelle
};

// Reihenfolge = Reihenfolge der Werte im Sample und der Spalten in der CSV.
// minVal/maxVal legen den Anzeigebereich des Diagramms in der Weboberflaeche fest.
// AI: +-20 V entspricht dem Messbereich der Eingangsstufe bei MCP_PGA = 1
// (bei MCP_PGA = 2 hier -10/+10 eintragen).
const Channel channels[] = {
  { "DI 1 (24V)",   CH_DIGITAL, 0, 1,   "",  0, SRC_DI, 0 },
  { "DI 2 (24V)",   CH_DIGITAL, 0, 1,   "",  0, SRC_DI, 1 },
  { "DI 3 (24V)",   CH_DIGITAL, 0, 1,   "",  0, SRC_DI, 2 },
  { "DI 4 (24V)",   CH_DIGITAL, 0, 1,   "",  0, SRC_DI, 3 },
  { "AI 1 (+-20V)", CH_ANALOG, -20, 20, "V", 2, SRC_AI, 0 },
  { "AI 2 (+-20V)", CH_ANALOG, -20, 20, "V", 2, SRC_AI, 1 },
  { "AI 3 (+-20V)", CH_ANALOG, -20, 20, "V", 2, SRC_AI, 2 },
  { "AI 4 (+-20V)", CH_ANALOG, -20, 20, "V", 2, SRC_AI, 3 },
  { "L1-N (230V~)", CH_ANALOG,  0, 260, "V", 0, SRC_AC, 0 },
  { "L2-N (230V~)", CH_ANALOG,  0, 260, "V", 0, SRC_AC, 1 },
  { "L3-N (230V~)", CH_ANALOG,  0, 260, "V", 0, SRC_AC, 2 },
};
const uint8_t NUM_CHANNELS = sizeof(channels) / sizeof(channels[0]);

// Ohne Padding, damit die Bytes 1:1 als BLE-Nutzlast verschickt werden
// koennen und in index.html mit DataView exakt so wieder eingelesen
// werden: [uint32 t_ms][float32 v0][float32 v1]...
#pragma pack(push, 1)
struct Sample {
  uint32_t t_ms;
  float    values[NUM_CHANNELS];
};

struct MarkerRecord {
  uint16_t id;
  uint32_t t_ms;
  char     label[12];   // nullterminiert
};

// Geraetestatus, wird jede Sekunde per Notify verschickt (STATUS-Characteristic).
// Die ersten 20 Byte passen auch in die kleinste MTU.
struct StatusPacket {
  uint8_t  version;         // 3 (Firmware 1.1: logSessionKB angehaengt, 1.2: Nullabgleich angehaengt)
  uint8_t  flags;           // Bit0 Karte steckt, Bit1 Karte bereit, Bit2 Aufzeichnung laeuft,
                            // Bit3 Akku schwach, Bit4 Akku leer, Bit5 Live-Puffer laeuft,
                            // Bit6 MCP3424 ok, Bit7 ADS7828 ok
  uint16_t battery_mV;
  uint32_t logIntervalMs;
  uint32_t logRows;         // Zeilen seit Start der Aufzeichnung
  uint32_t logKB;           // Groesse der aktuellen Datei
  uint32_t freeMB;
  uint32_t totalMB;
  uint32_t logSeconds;      // Dauer der laufenden Aufzeichnung
  uint32_t droppedRows;     // verlorene Zeilen (Karte zu langsam)
  uint8_t  sdError;         // siehe SDERR_*
  uint8_t  flags2;          // Bit0 Uhrzeit bekannt
  char     fileName[32];    // aktuelle bzw. letzte Datei
  uint32_t logSessionKB;    // geschriebene Datenmenge seit Start der Aufzeichnung (ueber alle Dateien)
  uint8_t  calSeq;          // zaehlt bei jedem abgeschlossenen Nullabgleich um 1 hoch
  uint8_t  calMask;         // Bit0..3: AI1..AI4 wurden beim letzten Nullabgleich uebernommen
  int16_t  aiZero_mV[4];    // gespeicherte Nullpunkt-Korrektur je Kanal in mV
};
#pragma pack(pop)

// Ein Eintrag fuer die SD-Aufzeichnung: Sample + Zusatzinfos
struct LogItem {
  Sample   s;
  int64_t  upMs;       // Laufzeit in ms als 64 Bit (fuer Datum/Uhrzeit)
  uint16_t markerId;   // 0 = kein Marker in dieser Zeile
};

enum : uint8_t {   // sdError
  SDERR_NONE = 0, SDERR_MOUNT = 1, SDERR_OPEN = 2, SDERR_WRITE = 3,
  SDERR_FULL = 4, SDERR_REMOVED = 5, SDERR_BATTERY = 6, SDERR_NO_CARD = 7
};
enum : uint8_t {   // Status in der Endmeldung der FILES-Characteristic
  FERR_OK = 0, FERR_NO_CARD = 1, FERR_NOT_FOUND = 2, FERR_BUSY = 3,
  FERR_CANCELLED = 4, FERR_READ = 5, FERR_DELETE = 6
};

const uint8_t FILE_NAME_MAX = 40;
enum SdCmdType : uint8_t { SDC_LOG_START, SDC_LOG_STOP, SDC_LIST, SDC_DOWNLOAD, SDC_DELETE };
struct SdCmd {
  SdCmdType type;
  uint16_t  conn;
  uint16_t  mtu;
  uint32_t  arg;
  char      name[FILE_NAME_MAX];
};

enum BleJobType : uint8_t { JOB_CHANNELS, JOB_HISTORY };
struct BleJob {
  BleJobType type;
  uint16_t   conn;
  uint16_t   mtu;
};

// ====================================================================
//  Globaler Zustand
// ====================================================================
// Kurze kritische Abschnitte fuer alles, was von mehreren Tasks angefasst wird
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// Live-Ringpuffer
Sample   ringBuffer[BUFFER_SIZE];
uint32_t bufHead  = 0;          // laufende Nummer des naechsten Samples
uint32_t bufCount = 0;          // gueltige Samples im Puffer
volatile bool recording = true;

// Marker
const uint8_t MAX_MARKERS = 20;
MarkerRecord  markerList[MAX_MARKERS];
uint8_t       markerCount  = 0;
uint16_t      nextMarkerId = 1;
volatile bool     markersPending   = false;
volatile uint16_t pendingLogMarker = 0;   // Marker, der in die naechste CSV-Zeile soll

// Messwerte (werden vom ADC-Task geschrieben, vom Sample-Task gelesen)
volatile float aiVolts[4] = { 0, 0, 0, 0 };
volatile float acVolts[3] = { 0, 0, 0 };
volatile bool  mcpOk = false;
volatile bool  adsOk = false;
volatile uint8_t acWindowSamples = 0;   // Diagnose: Abtastungen je Effektivwert-Fenster (Soll: 20)

// Nullabgleich der Analogeingaenge (Werte liegen dauerhaft im Flash, Schluessel "aiZero")
volatile float   aiZeroV[4] = { 0, 0, 0, 0 };   // wird von jedem AI-Messwert abgezogen
volatile bool    calRequest     = false;        // Weboberflaeche hat einen Nullabgleich angefordert
volatile bool    calSavePending = false;        // Abgleich fertig -> loop() speichert und meldet
volatile uint8_t calSeq  = 0;
volatile uint8_t calMask = 0;

// Akku
volatile uint16_t batteryMv   = 0;
volatile bool     batLow      = false;
volatile bool     batCritical = false;

// Uhrzeit: lokale Zeit (ms seit 1970) zum Zeitpunkt Laufzeit = 0; 0 = unbekannt
int64_t bootEpochLocalMs = 0;

// SD-Karte / Langzeitmessung (geschrieben vom SD-Task)
volatile bool     sdPresent     = false;
volatile bool     sdMounted     = false;
volatile bool     logActive     = false;
volatile uint8_t  sdError       = SDERR_NONE;
volatile uint32_t logIntervalMs = LOG_DEFAULT_INTERVAL_MS;
volatile uint32_t logRows       = 0;
volatile uint32_t logDropped    = 0;
volatile uint32_t logFileBytes  = 0;
volatile uint32_t logSessionKB  = 0;
volatile uint32_t logStartMs    = 0;
volatile uint32_t sdFreeMB      = 0;
volatile uint32_t sdTotalMB     = 0;
volatile bool     fileCancel    = false;
char     logFileName[32] = "";
File     logFile;
uint64_t sdFreeBytes  = 0;
uint32_t rowsInFile   = 0;
uint32_t logSessionRem = 0;     // Rest < 1 kB zu logSessionKB
uint32_t lastFlushMs  = 0;

Preferences prefs;

QueueHandle_t logQueue    = nullptr;   // LogItem: Sample-Task -> SD-Task
QueueHandle_t sdCmdQueue  = nullptr;   // SdCmd:   BLE -> SD-Task
QueueHandle_t bleJobQueue = nullptr;   // BleJob:  BLE-Callbacks -> loop()

// BLE
NimBLEServer*         pServer       = nullptr;
NimBLECharacteristic* pChannelsChar = nullptr;
NimBLECharacteristic* pSampleChar   = nullptr;
NimBLECharacteristic* pControlChar  = nullptr;
NimBLECharacteristic* pHistoryChar  = nullptr;
NimBLECharacteristic* pMarkersChar  = nullptr;
NimBLECharacteristic* pStatusChar   = nullptr;
NimBLECharacteristic* pFilesChar    = nullptr;
String   channelsJson;   // einmal beim Start berechnet
uint16_t connList[MAX_BLE_CONNECTIONS];
uint8_t  connCount = 0;

// ====================================================================
//  Kleine Helfer
// ====================================================================
static inline int64_t uptimeMs() {
  return esp_timer_get_time() / 1000;
}

static int64_t getBootEpoch() {
  portENTER_CRITICAL(&stateMux);
  int64_t v = bootEpochLocalMs;
  portEXIT_CRITICAL(&stateMux);
  return v;
}

// Uhrzeit vom Handy/Laptop uebernehmen. epochMsUtc = Date.now(),
// tzOffsetMin = Minuten, die die Ortszeit vor UTC liegt (MEZ: 60, MESZ: 120).
static void setClock(int64_t epochMsUtc, int16_t tzOffsetMin) {
  if (epochMsUtc < 1700000000000LL) return;   // unplausibel (vor Ende 2023)
  int64_t localMs = epochMsUtc + (int64_t)tzOffsetMin * 60000LL;
  int64_t boot = localMs - uptimeMs();
  portENTER_CRITICAL(&stateMux);
  bootEpochLocalMs = boot;
  portEXIT_CRITICAL(&stateMux);
  // Systemuhr auf Ortszeit stellen, damit auch die Datei-Zeitstempel auf der Karte stimmen
  struct timeval tv;
  tv.tv_sec  = (time_t)(localMs / 1000);
  tv.tv_usec = (suseconds_t)((localMs % 1000) * 1000);
  settimeofday(&tv, nullptr);
}

static void connAdd(uint16_t h) {
  portENTER_CRITICAL(&stateMux);
  bool known = false;
  for (uint8_t i = 0; i < connCount; i++) if (connList[i] == h) known = true;
  if (!known && connCount < MAX_BLE_CONNECTIONS) connList[connCount++] = h;
  portEXIT_CRITICAL(&stateMux);
}

static void connRemove(uint16_t h) {
  portENTER_CRITICAL(&stateMux);
  for (uint8_t i = 0; i < connCount; i++) {
    if (connList[i] == h) {
      connList[i] = connList[connCount - 1];
      connCount--;
      break;
    }
  }
  portEXIT_CRITICAL(&stateMux);
}

static bool connAlive(uint16_t h) {
  bool alive = false;
  portENTER_CRITICAL(&stateMux);
  for (uint8_t i = 0; i < connCount; i++) if (connList[i] == h) alive = true;
  portEXIT_CRITICAL(&stateMux);
  return alive;
}

static uint8_t connSnapshot(uint16_t* out) {
  portENTER_CRITICAL(&stateMux);
  uint8_t n = connCount;
  for (uint8_t i = 0; i < n; i++) out[i] = connList[i];
  portEXIT_CRITICAL(&stateMux);
  return n;
}

// Schickt ein Notify-Paket an genau ein verbundenes Geraet. Ist die
// Sendewarteschlange voll, wird gewartet und erneut versucht, statt das Paket
// stillschweigend zu verlieren. Laenge 0 = leeres Paket (Ende-Markierung).
static bool bleSend(NimBLECharacteristic* ch, uint16_t conn, const uint8_t* data, size_t len, uint32_t maxWaitMs) {
  if (!ch) return false;
  uint32_t t0 = millis();
  for (;;) {
    bool ok;
    if (len > 0) {
      ok = ch->notify(data, len, conn);
    } else {
      ch->setValue((const uint8_t*)"", 0);
      ok = ch->notify(conn);
    }
    if (ok) return true;
    if (!connAlive(conn) || millis() - t0 > maxWaitMs) return false;
    delay(5);
  }
}

// ====================================================================
//  I2C: MCP3424 (AI1..AI4) und ADS7828 (Netzspannung)
// ====================================================================
static_assert(MCP_RES_BITS == 12 || MCP_RES_BITS == 14 || MCP_RES_BITS == 16, "MCP_RES_BITS muss 12, 14 oder 16 sein");
static_assert(MCP_PGA == 1 || MCP_PGA == 2 || MCP_PGA == 4 || MCP_PGA == 8, "MCP_PGA muss 1, 2, 4 oder 8 sein");

// Konfigurationsbyte: Bit7 = Wandlung starten, Bit6..5 = Kanal, Bit4 = 0 (Einzelwandlung),
// Bit3..2 = Aufloesung, Bit1..0 = Verstaerkung
static const uint8_t  MCP_CFG_BASE = (uint8_t)((((MCP_RES_BITS - 12) / 2) << 2) | (MCP_PGA == 8 ? 3 : MCP_PGA == 4 ? 2 : MCP_PGA == 2 ? 1 : 0));
static const float    MCP_LSB_V    = 4.096f / (float)(1UL << MCP_RES_BITS) / (float)MCP_PGA;   // Referenz 2,048 V, +-Vollausschlag
static const uint32_t MCP_CONV_US  = (MCP_RES_BITS == 12) ? 5000 : (MCP_RES_BITS == 14) ? 20000 : 80000;

// ADS7828-Kommandobyte: single-ended, Kanal 0..3, interne Referenz an, Wandler an
static const uint8_t ADS_CMD[4] = { 0x8C, 0xCC, 0x9C, 0xDC };
static const float   AC_VOLT_PER_COUNT = AC_VREF / 4096.0f * AC_R_PRIMARY / AC_R_BURDEN;   // ca. 0,27 V je Digit

static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool mcpStart(uint8_t ch) {
  Wire.beginTransmission(MCP3424_ADDR);
  Wire.write((uint8_t)(0x80 | (ch << 5) | MCP_CFG_BASE));
  return Wire.endTransmission() == 0;
}

// Rueckgabe: 1 = neuer Wert, 0 = Wandlung noch nicht fertig, -1 = Busfehler
static int8_t mcpRead(int16_t& raw) {
  if (Wire.requestFrom(MCP3424_ADDR, (size_t)3) != 3) return -1;
  uint8_t hi  = Wire.read();
  uint8_t lo  = Wire.read();
  uint8_t cfg = Wire.read();
  if (cfg & 0x80) return 0;                 // RDY-Bit gesetzt = noch kein neuer Wert
  raw = (int16_t)(((uint16_t)hi << 8) | lo);  // bei 12/14 Bit bereits vorzeichenerweitert
  return 1;
}

static bool adsRead(uint8_t idx, uint16_t& val) {
  Wire.beginTransmission(ADS7828_ADDR);
  Wire.write(ADS_CMD[idx]);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADS7828_ADDR, (size_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  val = (uint16_t)(((hi & 0x0F) << 8) | lo);
  return true;
}

static bool adsInit() {
  if (!i2cProbe(ADS7828_ADDR)) return false;
  uint16_t dummy;
  if (!adsRead(0, dummy)) return false;   // schaltet die interne Referenz ein
  vTaskDelay(pdMS_TO_TICKS(10));          // Referenz einschwingen lassen
  return true;
}

// Liest laufend beide ADCs. Die Netzspannung wird mit 500 Hz je Phase
// abgetastet und daraus alle 40 ms der Effektivwert gebildet (Gleichanteil
// wird herausgerechnet). Der MCP3424 wandelt reihum einen Kanal nach dem
// anderen; zwischen Start und Abholen laeuft die Netzabtastung weiter.
void adcTask(void*) {
  uint32_t acN = 0;
  uint32_t acSum[3] = { 0, 0, 0 };
  uint64_t acSq[3]  = { 0, 0, 0 };
  int64_t  acWindowStart = esp_timer_get_time();
  uint8_t  adsFail = 0;

  uint8_t  mcpCh = 0, mcpFail = 0;
  bool     mcpBusy = false;
  uint32_t mcpT0 = 0;
  uint32_t lastProbe = 0;

  bool     calActive = false;          // Nullabgleich laeuft
  float    calSum[4] = { 0, 0, 0, 0 };
  uint8_t  calN[4]   = { 0, 0, 0, 0 };
  uint32_t calT0 = 0;

  adsOk = adsInit();
  mcpOk = i2cProbe(MCP3424_ADDR);
  Serial.printf("ADC: ADS7828 (Netzspannung) %s, MCP3424 (AI) %s\n", adsOk ? "ok" : "FEHLT", mcpOk ? "ok" : "FEHLT");

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(AC_SAMPLE_PERIOD_MS)) == pdFALSE) {
      // im Verzug (z.B. Busfehler): neu aufsetzen und anderen Tasks Luft lassen
      lastWake = xTaskGetTickCount();
      vTaskDelay(1);
    }

    // ---- Netzspannung L1..L3 ----
    if (adsOk) {
      uint16_t v[3];
      bool ok = true;
      for (uint8_t c = 0; c < 3 && ok; c++) ok = adsRead(c, v[c]);
      if (ok) {
        adsFail = 0;
        for (uint8_t c = 0; c < 3; c++) {
          acSum[c] += v[c];
          acSq[c]  += (uint32_t)v[c] * v[c];
        }
        acN++;
        if (esp_timer_get_time() - acWindowStart >= (int64_t)AC_WINDOW_MS * 1000 - 500) {
          for (uint8_t c = 0; c < 3; c++) {
            double mean = (double)acSum[c] / acN;
            double var  = (double)acSq[c] / acN - mean * mean;
            float volts = (var > 0) ? (float)sqrt(var) * AC_VOLT_PER_COUNT * AC_CAL[c] : 0;
            acVolts[c] = (volts < AC_ZERO_BELOW_V) ? 0 : volts;
            acSum[c] = 0;
            acSq[c]  = 0;
          }
          acWindowSamples = (acN > 255) ? 255 : (uint8_t)acN;
          acN = 0;
          acWindowStart = esp_timer_get_time();
        }
      } else {
        acN = 0;
        for (uint8_t c = 0; c < 3; c++) { acSum[c] = 0; acSq[c] = 0; }
        acWindowStart = esp_timer_get_time();
        if (++adsFail >= 10) {
          adsOk = false;
          for (uint8_t c = 0; c < 3; c++) acVolts[c] = 0;
          Serial.println("WARNUNG: ADS7828 antwortet nicht mehr - Netzspannungen liefern 0.");
        }
      }
    }

    // ---- Analogeingaenge AI1..AI4 ----
    if (mcpOk) {
      if (!mcpBusy) {
        if (mcpStart(mcpCh)) { mcpBusy = true; mcpT0 = micros(); }
        else mcpFail++;
      } else if (micros() - mcpT0 >= MCP_CONV_US) {
        int16_t raw = 0;
        int8_t r = mcpRead(raw);
        if (r == 1) {
          float v = raw * MCP_LSB_V * AI_DIVIDER * AI_GAIN_CAL[mcpCh] - AI_OFFSET_V[mcpCh];   // ohne Nullabgleich
          if (calActive && calN[mcpCh] < AI_ZERO_SAMPLES) { calSum[mcpCh] += v; calN[mcpCh]++; }
          aiVolts[mcpCh] = v - aiZeroV[mcpCh];
          mcpFail = 0;
          mcpCh = (mcpCh + 1) & 3;
          mcpBusy = false;
          if (mcpStart(mcpCh)) { mcpBusy = true; mcpT0 = micros(); }   // gleich den naechsten Kanal anstossen
        } else if (r < 0 || micros() - mcpT0 > 4 * MCP_CONV_US) {
          mcpBusy = false;
          mcpFail++;
        }
      }
      if (mcpFail >= 10) {
        mcpOk = false;
        mcpBusy = false;
        for (uint8_t c = 0; c < 4; c++) aiVolts[c] = 0;
        Serial.println("WARNUNG: MCP3424 antwortet nicht mehr - AI-Kanaele liefern 0.");
      }
    }

    // ---- Nullabgleich: Mittelwert je Kanal bei kurzgeschlossenen Eingaengen ----
    if (calRequest) {
      calRequest = false;
      for (uint8_t c = 0; c < 4; c++) { calSum[c] = 0; calN[c] = 0; }
      calActive = true;
      calT0 = millis();
    }
    if (calActive) {
      bool complete = true;
      for (uint8_t c = 0; c < 4; c++) if (calN[c] < AI_ZERO_SAMPLES) complete = false;
      if (complete) {
        uint8_t mask = 0;
        for (uint8_t c = 0; c < 4; c++) {
          float mean = calSum[c] / calN[c];
          if (fabsf(mean) <= AI_ZERO_MAX_V) {   // sonst: Eingang offenbar nicht kurzgeschlossen -> alten Wert behalten
            aiZeroV[c] = mean;
            mask |= (uint8_t)(1 << c);
          }
        }
        calMask = mask;
        calActive = false;
        calSavePending = true;
      } else if (millis() - calT0 > 5000) {   // ADC liefert keine Werte
        calMask = 0;
        calActive = false;
        calSavePending = true;
      }
    }

    // ---- fehlende Bausteine einmal pro Sekunde neu suchen ----
    if ((!adsOk || !mcpOk) && millis() - lastProbe >= 1000) {
      lastProbe = millis();
      if (!adsOk) { adsOk = adsInit(); adsFail = 0; acWindowStart = esp_timer_get_time(); }
      if (!mcpOk) { mcpOk = i2cProbe(MCP3424_ADDR); mcpFail = 0; }
      lastWake = xTaskGetTickCount();
    }
  }
}

// ====================================================================
//  Kanal-Erfassung, Ringpuffer, Live-Uebertragung
// ====================================================================
void readChannels(float* out) {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    const Channel& c = channels[i];
    switch (c.src) {
      case SRC_DI: out[i] = (digitalRead(PIN_DI[c.index]) == HIGH) ? 1.0f : 0.0f; break;
      case SRC_AI: out[i] = aiVolts[c.index]; break;
      case SRC_AC: out[i] = acVolts[c.index]; break;
    }
  }
}

// Zustand aller "schaltenden" Signale als Bitmuster: Bit0..3 = DI1..DI4,
// Bit4..6 = Netzspannung L1..L3 vorhanden. Aendert sich das Muster, wird
// beim Loggen sofort eine Zeile geschrieben.
static uint8_t eventBits(const float* v) {
  uint8_t bits = 0;
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    const Channel& c = channels[i];
    if (c.src == SRC_DI && v[i] > 0.5f) bits |= (uint8_t)(0x01 << c.index);
    if (c.src == SRC_AC && v[i] > AC_EVENT_THRESHOLD_V) bits |= (uint8_t)(0x10 << c.index);
  }
  return bits;
}

// Fester 40-ms-Takt, unabhaengig davon, was BLE oder die SD-Karte gerade tun.
void sampleTask(void*) {
  int64_t lastLogMs  = 0;
  uint8_t lastEvents = 0;
  bool    wasLogging = false;

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));

    LogItem item;
    item.upMs     = uptimeMs();
    item.s.t_ms   = (uint32_t)item.upMs;   // identisch zu millis()
    item.markerId = 0;
    readChannels(item.s.values);

    // ---- Live-Puffer und Live-Uebertragung ----
    if (recording) {
      portENTER_CRITICAL(&stateMux);
      ringBuffer[bufHead % BUFFER_SIZE] = item.s;
      bufHead++;
      if (bufCount < BUFFER_SIZE) bufCount++;
      portEXIT_CRITICAL(&stateMux);

      uint16_t conns[MAX_BLE_CONNECTIONS];
      uint8_t n = connSnapshot(conns);
      for (uint8_t i = 0; i < n; i++) {
        pSampleChar->notify((const uint8_t*)&item.s, sizeof(Sample), conns[i]);
      }
    }

    // ---- Langzeitmessung auf SD (laeuft unabhaengig vom Live-Puffer) ----
    if (logActive) {
      uint8_t events = eventBits(item.s.values);
      bool first   = !wasLogging;
      bool due     = first || (item.upMs - lastLogMs) >= (int64_t)logIntervalMs - (int64_t)(SAMPLE_INTERVAL_MS / 2);
      bool changed = LOG_ON_CHANGE && !first && events != lastEvents;
      uint16_t marker = 0;
      if (pendingLogMarker) {
        portENTER_CRITICAL(&stateMux);
        marker = pendingLogMarker;
        pendingLogMarker = 0;
        portEXIT_CRITICAL(&stateMux);
      }
      if (due || changed || marker) {
        item.markerId = marker;
        if (xQueueSend(logQueue, &item, 0) != pdTRUE) logDropped = logDropped + 1;
        if (due) lastLogMs = item.upMs;
      }
      lastEvents = events;
      wasLogging = true;
    } else {
      wasLogging = false;
      pendingLogMarker = 0;   // Marker ausserhalb einer Aufzeichnung nicht in die naechste Datei mitnehmen
    }
  }
}

// ====================================================================
//  SD-Karte: Langzeitmessung und Dateizugriff (alles im SD-Task)
// ====================================================================
static void sdUpdateSpace() {
  uint64_t total = SD_MMC.totalBytes();
  uint64_t used  = SD_MMC.usedBytes();
  sdFreeBytes = (total > used) ? (total - used) : 0;
  sdTotalMB   = (uint32_t)(total / (1024ULL * 1024ULL));
  sdFreeMB    = (uint32_t)(sdFreeBytes / (1024ULL * 1024ULL));
}

static bool sdMount() {
  bool ok = SD_MMC.begin("/sdcard", false, false, SDMMC_FREQ_HIGHSPEED);
  if (!ok) ok = SD_MMC.begin("/sdcard", false, false, SDMMC_FREQ_DEFAULT);   // zweiter Versuch mit 20 MHz
  if (!ok) return false;
  if (SD_MMC.cardType() == CARD_NONE) {
    SD_MMC.end();
    return false;
  }
  sdUpdateSpace();
  Serial.printf("SD-Karte bereit: %lu MB gesamt, %lu MB frei\n", (unsigned long)sdTotalMB, (unsigned long)sdFreeMB);
  return true;
}

// Haengt eine Zahl an, auf Wunsch mit Dezimalkomma
static char* appendNumber(char* p, char* end, float v, uint8_t decimals) {
  char tmp[24];
  int n = snprintf(tmp, sizeof(tmp), "%.*f", decimals, (double)v);
  if (n > (int)sizeof(tmp) - 1) n = sizeof(tmp) - 1;
  for (int i = 0; i < n && p < end; i++) {
    *p++ = (CSV_DECIMAL_COMMA && tmp[i] == '.') ? ',' : tmp[i];
  }
  return p;
}

static size_t formatHeader(char* out, size_t cap) {
  int n = snprintf(out, cap, "Zeit%ct_ms", CSV_SEPARATOR);
  for (uint8_t i = 0; i < NUM_CHANNELS && n < (int)cap; i++) {
    // Einheit in eckigen Klammern anhaengen, z.B. "AI 1 (0-10V) [V]"
    if (channels[i].unit[0]) n += snprintf(out + n, cap - n, "%c%s [%s]", CSV_SEPARATOR, channels[i].name, channels[i].unit);
    else                     n += snprintf(out + n, cap - n, "%c%s", CSV_SEPARATOR, channels[i].name);
  }
  if (n < (int)cap) n += snprintf(out + n, cap - n, "%cMarker\r\n", CSV_SEPARATOR);
  return (n < (int)cap) ? (size_t)n : cap - 1;
}

// Eine CSV-Zeile: Zeit;t_ms;DI1..;AI1..;L1..;Marker
static size_t formatRow(const LogItem& it, char* out, size_t cap) {
  char* p   = out;
  char* end = out + cap - 12;   // Platz fuer Marker und Zeilenende freihalten
  int64_t boot = getBootEpoch();
  if (boot != 0) {
    int64_t absMs = boot + it.upMs;
    time_t  sec   = (time_t)(absMs / 1000);
    struct tm tmv;
    gmtime_r(&sec, &tmv);
    p += snprintf(p, end - p, "%04d-%02d-%02d %02d:%02d:%02d%c%03d",
                  tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec,
                  CSV_DECIMAL_COMMA ? ',' : '.', (int)(absMs % 1000));
  }
  p += snprintf(p, end - p, "%c%lu", CSV_SEPARATOR, (unsigned long)it.s.t_ms);
  for (uint8_t i = 0; i < NUM_CHANNELS && p < end; i++) {
    *p++ = CSV_SEPARATOR;
    if (channels[i].type == CH_DIGITAL) {
      *p++ = (it.s.values[i] > 0.5f) ? '1' : '0';
    } else {
      p = appendNumber(p, end, it.s.values[i], channels[i].decimals + 1);
    }
  }
  if (p > end) p = end;
  *p++ = CSV_SEPARATOR;
  if (it.markerId) p += snprintf(p, 8, "M%u", (unsigned)it.markerId);
  *p++ = '\r';
  *p++ = '\n';
  return (size_t)(p - out);
}

static void logStop(uint8_t err, bool clearResume) {
  bool wasActive = logActive;
  logActive = false;
  if (wasActive && logFile) {
    logFile.flush();
    logFile.close();
  }
  sdError = err;
  if (clearResume) prefs.putBool("logOn", false);
  if (wasActive) {
    Serial.printf("Langzeitmessung beendet (%s), %lu Zeilen, Code %u\n", logFileName, (unsigned long)logRows, err);
    if (sdMounted && err != SDERR_REMOVED) sdUpdateSpace();
  }
}

static bool logOpenFile() {
  char name[80];
  char path[84];
  int64_t boot = getBootEpoch();
  if (boot != 0) {
    time_t sec = (time_t)((boot + uptimeMs()) / 1000);
    struct tm tmv;
    gmtime_r(&sec, &tmv);
    snprintf(name, sizeof(name), "LOG_%04d%02d%02d_%02d%02d%02d.csv",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
    snprintf(path, sizeof(path), "/%s", name);
    for (uint8_t k = 2; SD_MMC.exists(path) && k < 100; k++) {
      snprintf(name, sizeof(name), "LOG_%04d%02d%02d_%02d%02d%02d_%u.csv",
               tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec, k);
      snprintf(path, sizeof(path), "/%s", name);
    }
  } else {
    // Uhrzeit unbekannt (noch kein Handy verbunden): fortlaufende Nummer
    uint32_t no = prefs.getUInt("fileNo", 0);
    do {
      no++;
      snprintf(name, sizeof(name), "LOG_%04lu.csv", (unsigned long)no);
      snprintf(path, sizeof(path), "/%s", name);
    } while (SD_MMC.exists(path) && no < 99999);
    prefs.putUInt("fileNo", no);
  }

  logFile = SD_MMC.open(path, FILE_WRITE);
  if (!logFile) return false;

  char header[512];
  size_t n = formatHeader(header, sizeof(header));
  if (logFile.write((const uint8_t*)header, n) != n) {
    logFile.close();
    return false;
  }
  logFile.flush();
  strlcpy(logFileName, name, sizeof(logFileName));
  logFileBytes = (uint32_t)n;
  sdFreeBytes  = (sdFreeBytes > n) ? sdFreeBytes - n : 0;
  rowsInFile   = 0;
  lastFlushMs  = millis();
  return true;
}

static void logStart(uint32_t intervalMs, bool saveForResume) {
  if (logActive) return;
  if (!sdMounted)  { sdError = sdPresent ? SDERR_MOUNT : SDERR_NO_CARD; return; }
  if (batCritical) { sdError = SDERR_BATTERY; return; }
  if (sdFreeBytes < LOG_MIN_FREE_BYTES) { sdError = SDERR_FULL; return; }
  if (intervalMs < SAMPLE_INTERVAL_MS) intervalMs = SAMPLE_INTERVAL_MS;
  intervalMs = ((intervalMs + SAMPLE_INTERVAL_MS - 1) / SAMPLE_INTERVAL_MS) * SAMPLE_INTERVAL_MS;   // auf das Messraster runden

  if (!logOpenFile()) { sdError = SDERR_OPEN; return; }
  logIntervalMs = intervalMs;
  logRows    = 0;
  logSessionKB  = 0;
  logSessionRem = 0;
  logDropped = 0;
  logStartMs = millis();
  sdError    = SDERR_NONE;
  xQueueReset(logQueue);
  logActive  = true;
  if (saveForResume) {
    prefs.putUInt("logInt", intervalMs);
    prefs.putBool("logOn", true);
  }
  Serial.printf("Langzeitmessung gestartet: %s, Intervall %lu ms\n", logFileName, (unsigned long)intervalMs);
}

static bool logWrite(const char* data, size_t len) {
  if (logFile.write((const uint8_t*)data, len) != len) {
    logStop(SDERR_WRITE, true);
    return false;
  }
  logFileBytes = logFileBytes + (uint32_t)len;
  logSessionRem += (uint32_t)len;
  logSessionKB  = logSessionKB + logSessionRem / 1024;
  logSessionRem %= 1024;
  sdFreeBytes  = (sdFreeBytes > len) ? sdFreeBytes - len : 0;
  return true;
}

// Holt wartende Zeilen aus der Queue und schreibt sie auf die Karte.
// wait = wie lange auf die erste Zeile gewartet wird.
static void logService(TickType_t wait) {
  static char batch[4096];
  size_t fill = 0;
  LogItem it;
  while (xQueueReceive(logQueue, &it, wait) == pdTRUE) {
    wait = 0;
    if (!logActive) continue;   // Reste nach einem Stopp verwerfen
    fill += formatRow(it, batch + fill, sizeof(batch) - fill);
    logRows = logRows + 1;
    rowsInFile++;
    if (fill > sizeof(batch) - 256) {
      if (!logWrite(batch, fill)) return;
      fill = 0;
    }
  }
  if (!logActive) return;
  if (fill > 0 && !logWrite(batch, fill)) return;

  uint32_t now = millis();
  if (now - lastFlushMs >= (batLow ? 1000u : LOG_FLUSH_MS)) {
    lastFlushMs = now;
    logFile.flush();   // inkl. fsync: Daten und Dateigroesse stehen danach sicher auf der Karte
    sdFreeMB = (uint32_t)(sdFreeBytes / (1024ULL * 1024ULL));
    if (sdFreeBytes < LOG_MIN_FREE_BYTES) {
      logStop(SDERR_FULL, true);
      return;
    }
  }

  if (rowsInFile >= LOG_MAX_ROWS_PER_FILE) {
    logFile.flush();
    logFile.close();
    if (!logOpenFile()) logStop(SDERR_OPEN, true);
  }
}

static bool validFileName(const char* name) {
  size_t n = strlen(name);
  if (n == 0 || n >= sizeof(logFileName)) return false;
  for (size_t i = 0; i < n; i++) {
    char c = name[i];
    if (c == '/' || c == '\\' || c < 0x20) return false;
  }
  return strstr(name, "..") == nullptr;
}

static bool isCsvName(const char* name) {
  size_t n = strlen(name);
  return n > 4 && n < sizeof(logFileName) && strcasecmp(name + n - 4, ".csv") == 0;
}

// Endmeldung auf der FILES-Characteristic: 'E', Vorgang (1 Liste, 2 Download, 3 Loeschen), Status, Wert
static void filesEnd(uint16_t conn, uint8_t op, uint8_t status, uint32_t value) {
  uint8_t p[7] = { 'E', op, status, 0, 0, 0, 0 };
  memcpy(p + 3, &value, 4);
  bleSend(pFilesChar, conn, p, sizeof(p), 2000);
}

// Dateiliste: je Datei ein Paket 'L', Flags (Bit0 = wird gerade beschrieben), Groesse (uint32), Name
static void sdList(const SdCmd& c) {
  if (!sdMounted) { filesEnd(c.conn, 1, FERR_NO_CARD, 0); return; }
  uint32_t count = 0;
  File root = SD_MMC.open("/");
  if (root) {
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      const char* name = f.name();
      if (!f.isDirectory() && isCsvName(name)) {
        bool active = logActive && strcmp(name, logFileName) == 0;
        uint32_t size = active ? (uint32_t)logFileBytes : (uint32_t)f.size();
        uint8_t p[6 + sizeof(logFileName)];
        size_t nl = strlen(name);
        p[0] = 'L';
        p[1] = active ? 1 : 0;
        memcpy(p + 2, &size, 4);
        memcpy(p + 6, name, nl);
        if (!bleSend(pFilesChar, c.conn, p, 6 + nl, 1000)) { f.close(); break; }
        count++;
        vTaskDelay(pdMS_TO_TICKS(8));
      }
      f.close();
      logService(0);   // Aufzeichnung laeuft waehrenddessen weiter
      if (count >= 300) break;
    }
    root.close();
  }
  filesEnd(c.conn, 1, FERR_OK, count);
}

// Datei-Download: 'S' + Groesse, dann 'D' + Daten ..., am Ende 'E'.
// Das Sendetempo passt sich an: wird die Sendewarteschlange voll, wird
// langsamer gesendet, laeuft es eine Weile glatt, wieder schneller.
static void sdDownload(const SdCmd& c) {
  if (!sdMounted)             { filesEnd(c.conn, 2, FERR_NO_CARD, 0);   return; }
  if (!validFileName(c.name)) { filesEnd(c.conn, 2, FERR_NOT_FOUND, 0); return; }
  if (logActive && strcmp(c.name, logFileName) == 0) { filesEnd(c.conn, 2, FERR_BUSY, 0); return; }

  char path[FILE_NAME_MAX + 2];
  snprintf(path, sizeof(path), "/%s", c.name);
  File f = SD_MMC.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    filesEnd(c.conn, 2, FERR_NOT_FOUND, 0);
    return;
  }

  uint32_t size = (uint32_t)f.size();
  uint8_t hdr[5] = { 'S', 0, 0, 0, 0 };
  memcpy(hdr + 1, &size, 4);
  bleSend(pFilesChar, c.conn, hdr, sizeof(hdr), 2000);

  static uint8_t buf[244];
  uint16_t mtu   = (c.mtu >= 23) ? c.mtu : 23;
  uint16_t chunk = mtu - 3 - 1;
  if (chunk > sizeof(buf) - 1) chunk = sizeof(buf) - 1;
  buf[0] = 'D';

  uint32_t sent   = 0;
  uint8_t  status = FERR_OK;
  uint16_t paceMs = 10, okRun = 0;
  fileCancel = false;
  uint32_t tStart = millis();

  while (sent < size) {
    int n = f.read(buf + 1, chunk);
    if (n <= 0) { status = FERR_READ; break; }

    uint32_t t0 = millis();
    bool ok;
    while (!(ok = pFilesChar->notify(buf, (size_t)n + 1, c.conn))) {
      if (fileCancel || !connAlive(c.conn) || millis() - t0 > 4000) break;
      okRun = 0;
      if (paceMs < 40) paceMs += 2;
      logService(pdMS_TO_TICKS(40));   // Warteschlange voll: abwarten, dabei die Aufzeichnung bedienen
    }
    if (!ok || fileCancel) { status = FERR_CANCELLED; break; }

    sent += (uint32_t)n;
    if (++okRun >= 50) { okRun = 0; if (paceMs > 4) paceMs--; }
    logService(0);
    vTaskDelay(pdMS_TO_TICKS(paceMs));
  }
  f.close();
  Serial.printf("Download %s: %lu von %lu Byte in %lu ms, Status %u\n", c.name, (unsigned long)sent,
                (unsigned long)size, (unsigned long)(millis() - tStart), status);
  if (connAlive(c.conn)) filesEnd(c.conn, 2, status, sent);
}

static void sdDelete(const SdCmd& c) {
  uint8_t status = FERR_OK;
  if (!sdMounted) status = FERR_NO_CARD;
  else if (!validFileName(c.name)) status = FERR_NOT_FOUND;
  else if (logActive && strcmp(c.name, logFileName) == 0) status = FERR_BUSY;
  else {
    char path[FILE_NAME_MAX + 2];
    snprintf(path, sizeof(path), "/%s", c.name);
    if (!SD_MMC.exists(path)) status = FERR_NOT_FOUND;
    else if (!SD_MMC.remove(path)) status = FERR_DELETE;
    else if (!logActive) sdUpdateSpace();
  }
  filesEnd(c.conn, 3, status, 0);
}

void sdTask(void*) {
  bool     cdStable = false;
  uint8_t  cdCount  = 0;
  uint32_t nextMountTry = 0;
  bool     mountWarned  = false;
  bool     resumePending = LOG_AUTO_RESUME && prefs.getBool("logOn", false);
  uint32_t resumeInterval = prefs.getUInt("logInt", LOG_DEFAULT_INTERVAL_MS);

  for (;;) {
    // ---- Kartenerkennung (entprellt) ----
    bool raw = SD_USE_CARD_DETECT ? (digitalRead(PIN_SD_CD) == (SD_CD_ACTIVE_LOW ? LOW : HIGH)) : true;
    if (raw != cdStable) {
      if (++cdCount >= 4) { cdStable = raw; cdCount = 0; }
    } else {
      cdCount = 0;
    }
    sdPresent = cdStable;

    if (sdPresent && !sdMounted && (int32_t)(millis() - nextMountTry) >= 0) {
      if (sdMount()) {
        sdMounted = true;
        mountWarned = false;
        if (sdError == SDERR_MOUNT || sdError == SDERR_NO_CARD || sdError == SDERR_REMOVED) sdError = SDERR_NONE;
      } else {
        sdError = SDERR_MOUNT;
        nextMountTry = millis() + 3000;
        if (!mountWarned) {
          mountWarned = true;
          Serial.println("WARNUNG: SD-Karte nicht lesbar (FAT32-formatiert? richtig eingerastet?)");
        }
      }
    }
    if (!sdPresent && sdMounted) {
      // Karte gezogen: laufende Aufzeichnung ist damit beendet
      if (logActive) logStop(SDERR_REMOVED, true);
      SD_MMC.end();
      sdMounted = false;
      sdFreeBytes = 0;
      sdFreeMB = 0;
      sdTotalMB = 0;
      Serial.println("SD-Karte entfernt.");
    }
    if (!sdPresent) mountWarned = false;

    // ---- nach einem Akkuwechsel weiterloggen ----
    if (resumePending && sdMounted) {
      resumePending = false;
      Serial.println("Setze Langzeitmessung nach Neustart fort.");
      logStart(resumeInterval, false);
    } else if (resumePending && millis() > 15000) {
      // beim Start steckte keine lesbare Karte -> die alte Messung gilt als beendet
      resumePending = false;
      prefs.putBool("logOn", false);
    }

    // ---- Auftraege von der Weboberflaeche ----
    SdCmd cmd;
    while (xQueueReceive(sdCmdQueue, &cmd, 0) == pdTRUE) {
      switch (cmd.type) {
        case SDC_LOG_START: resumePending = false; logStart(cmd.arg, true); break;
        case SDC_LOG_STOP:  resumePending = false; logStop(SDERR_NONE, true); break;
        case SDC_LIST:      sdList(cmd); break;
        case SDC_DOWNLOAD:  sdDownload(cmd); break;
        case SDC_DELETE:    sdDelete(cmd); break;
      }
    }

    // ---- Akku leer: Datei sauber schliessen, bevor die Hardware abschaltet ----
    // (der Merker zum Fortsetzen bleibt gesetzt -> nach dem Akkuwechsel geht es weiter)
    if (batCritical && logActive) logStop(SDERR_BATTERY, false);

    logService(pdMS_TO_TICKS(50));
  }
}

// ====================================================================
//  BLE: Kanalbeschreibung, Verlauf, Marker, Status
// ====================================================================
void buildChannelsJson() {
  channelsJson.reserve(1200);
  channelsJson  = "{\"demo\":false,\"fw\":\"";
  channelsJson += FW_VERSION;
  channelsJson += "\",\"interval\":";
  channelsJson += String(SAMPLE_INTERVAL_MS);
  channelsJson += ",\"channels\":[";
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    char buf[160];
    snprintf(buf, sizeof(buf), "%s{\"name\":\"%s\",\"kind\":\"%s\",\"min\":%g,\"max\":%g,\"unit\":\"%s\",\"dec\":%u}",
             i ? "," : "", channels[i].name, channels[i].type == CH_DIGITAL ? "digital" : "analog",
             (double)channels[i].minVal, (double)channels[i].maxVal, channels[i].unit, channels[i].decimals);
    channelsJson += buf;
  }
  channelsJson += "]}";
}

// Schickt die Kanal-Beschreibung in Stuecken ueber Notify (das JSON ist
// laenger als eine MTU), gefolgt von einem leeren Paket als Ende-Markierung.
void sendChannels(uint16_t conn, uint16_t mtu) {
  uint16_t chunkSize = (mtu > 23) ? (mtu - 3) : 20;
  for (size_t offset = 0; offset < channelsJson.length(); offset += chunkSize) {
    size_t len = channelsJson.length() - offset;
    if (len > chunkSize) len = chunkSize;
    if (!bleSend(pChannelsChar, conn, (const uint8_t*)(channelsJson.c_str() + offset), len, 1000)) return;
    delay(15);
  }
  bleSend(pChannelsChar, conn, nullptr, 0, 1000);
}

// Schickt den Inhalt des Ringpuffers an ein frisch verbundenes Geraet.
// Die Messung laeuft waehrenddessen im Sample-Task weiter.
void sendHistory(uint16_t conn, uint16_t mtu) {
  uint8_t batchSize = (mtu > 3) ? (uint8_t)((mtu - 3) / sizeof(Sample)) : 1;
  if (batchSize < 1) batchSize = 1;
  if (batchSize > MAX_HISTORY_BATCH) batchSize = MAX_HISTORY_BATCH;

  portENTER_CRITICAL(&stateMux);
  uint32_t end = bufHead;
  uint32_t seq = bufHead - bufCount;
  portEXIT_CRITICAL(&stateMux);

  uint8_t  buf[MAX_HISTORY_BATCH * sizeof(Sample)];
  uint32_t sent = 0;
  while ((int32_t)(end - seq) > 0) {
    uint8_t count = 0;
    portENTER_CRITICAL(&stateMux);
    uint32_t oldest = bufHead - bufCount;
    if ((int32_t)(seq - oldest) < 0) seq = oldest;   // inzwischen ueberschrieben oder Puffer geleert
    while (count < batchSize && (int32_t)(end - seq) > 0) {
      memcpy(buf + count * sizeof(Sample), &ringBuffer[seq % BUFFER_SIZE], sizeof(Sample));
      seq++;
      count++;
    }
    portEXIT_CRITICAL(&stateMux);
    if (count == 0) break;
    if (!bleSend(pHistoryChar, conn, buf, count * sizeof(Sample), 1000)) return;   // Verbindung weg
    sent += count;
    delay(15);   // der Verbindung Zeit geben, das Paket tatsaechlich zu senden
  }
  bleSend(pHistoryChar, conn, nullptr, 0, 1000);
  Serial.printf("sendHistory: %lu Samples verschickt\n", (unsigned long)sent);
}

// Schickt die komplette Marker-Liste an ALLE verbundenen Geraete, damit
// Handy und Laptop denselben Stand zeigen.
void broadcastMarkers() {
  MarkerRecord copy[MAX_MARKERS];
  portENTER_CRITICAL(&stateMux);
  uint8_t n = markerCount;
  memcpy(copy, markerList, n * sizeof(MarkerRecord));
  portEXIT_CRITICAL(&stateMux);

  uint16_t conns[MAX_BLE_CONNECTIONS];
  uint8_t nc = connSnapshot(conns);
  for (uint8_t k = 0; k < nc; k++) {
    bool ok = true;
    for (uint8_t i = 0; i < n && ok; i++) {
      ok = bleSend(pMarkersChar, conns[k], (const uint8_t*)&copy[i], sizeof(MarkerRecord), 500);
      delay(10);
    }
    if (ok) bleSend(pMarkersChar, conns[k], nullptr, 0, 500);
  }
}

void fillStatus(StatusPacket& st) {
  memset(&st, 0, sizeof(st));
  st.version = 3;
  st.flags = (sdPresent ? 0x01 : 0) | (sdMounted ? 0x02 : 0) | (logActive ? 0x04 : 0) | (batLow ? 0x08 : 0)
           | (batCritical ? 0x10 : 0) | (recording ? 0x20 : 0) | (mcpOk ? 0x40 : 0) | (adsOk ? 0x80 : 0);
  st.battery_mV    = batteryMv;
  st.logIntervalMs = logIntervalMs;
  st.logRows       = logRows;
  st.logKB         = logFileBytes / 1024;
  st.freeMB        = sdFreeMB;
  st.totalMB       = sdTotalMB;
  st.logSeconds    = logActive ? (millis() - logStartMs) / 1000 : 0;
  st.droppedRows   = logDropped;
  st.sdError       = sdError;
  st.flags2        = (getBootEpoch() != 0) ? 0x01 : 0;
  strlcpy(st.fileName, logFileName, sizeof(st.fileName));
  st.logSessionKB  = logSessionKB;
  st.calSeq        = calSeq;
  st.calMask       = calMask;
  for (uint8_t c = 0; c < 4; c++) st.aiZero_mV[c] = (int16_t)lroundf(aiZeroV[c] * 1000.0f);
}

void printStatus() {
  StatusPacket st;
  fillStatus(st);
  Serial.printf("--- Messgeraet %s ---\n", FW_VERSION);
  Serial.printf("Akku: %.2f V%s%s\n", st.battery_mV / 1000.0, batLow ? " (schwach)" : "", batCritical ? " (LEER)" : "");
  Serial.printf("ADC: MCP3424 %s, ADS7828 %s\n", mcpOk ? "ok" : "FEHLT", adsOk ? "ok" : "FEHLT");
  Serial.printf("DI: %d %d %d %d\n", digitalRead(PIN_DI[0]), digitalRead(PIN_DI[1]), digitalRead(PIN_DI[2]), digitalRead(PIN_DI[3]));
  Serial.printf("AI: %.3f  %.3f  %.3f  %.3f V\n", (double)aiVolts[0], (double)aiVolts[1], (double)aiVolts[2], (double)aiVolts[3]);
  Serial.printf("AI-Nullabgleich: %.1f  %.1f  %.1f  %.1f mV\n", aiZeroV[0] * 1000.0, aiZeroV[1] * 1000.0, aiZeroV[2] * 1000.0, aiZeroV[3] * 1000.0);
  Serial.printf("Netz: L1 %.1f  L2 %.1f  L3 %.1f V (%u Abtastungen je %lu ms)\n", (double)acVolts[0], (double)acVolts[1],
                (double)acVolts[2], acWindowSamples, (unsigned long)AC_WINDOW_MS);
  Serial.printf("SD: Erkennung IO48=%d, %s, %lu MB frei von %lu MB, Fehlercode %u\n", digitalRead(PIN_SD_CD),
                sdMounted ? "bereit" : (sdPresent ? "nicht lesbar" : "keine Karte"),
                (unsigned long)st.freeMB, (unsigned long)st.totalMB, st.sdError);
  Serial.printf("Langzeitmessung: %s, Datei %s, %lu Zeilen, %lu kB, Intervall %lu ms, verloren %lu\n",
                logActive ? "laeuft" : "aus", st.fileName[0] ? st.fileName : "-", (unsigned long)st.logRows,
                (unsigned long)st.logKB, (unsigned long)st.logIntervalMs, (unsigned long)st.droppedRows);
  Serial.printf("BLE: %u Verbindung(en), Uhrzeit %s\n", connCount, (st.flags2 & 1) ? "bekannt" : "unbekannt");
}

// ---------- BLE-Callbacks ----------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    connAdd(connInfo.getConnHandle());
    // kurzes Verbindungsintervall anfragen (15..30 ms) -> fluessige Live-Ansicht, schnellerer Datei-Download
    server->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
    // Advertising weiterlaufen lassen, sonst ist das Geraet fuer ein zweites
    // Geraet (z.B. Laptop, waehrend das Handy schon verbunden ist) nicht auffindbar.
    if (server->getConnectedCount() < MAX_BLE_CONNECTIONS) {
      NimBLEDevice::startAdvertising();
    }
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    connRemove(connInfo.getConnHandle());
    NimBLEDevice::startAdvertising();
  }
};

// Befehle (Client -> ESP32), erstes Byte = Typ:
//   0x00 Live-Puffer anhalten, 0x01 fortsetzen, 0x02 Puffer leeren   (wie Prototyp)
//   0x10 Langzeitmessung starten, Bytes 1-4 = Intervall in ms (uint32, little endian)
//   0x11 Langzeitmessung stoppen
//   0x12 Uhrzeit setzen, Bytes 1-8 = ms seit 1970 UTC (int64), Bytes 9-10 = Zeitzonen-Offset in Minuten (int16)
//   0x13 Nullabgleich der Analogeingaenge (Eingaenge muessen kurzgeschlossen sein)
class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = pChar->getValue();
    size_t len = val.length();
    if (len == 0) return;
    const uint8_t* d = val.data();
    switch (d[0]) {
      case 0x00: recording = false; break;
      case 0x01: recording = true; break;
      case 0x02:
        portENTER_CRITICAL(&stateMux);
        bufCount = 0;
        portEXIT_CRITICAL(&stateMux);
        break;
      case 0x10: {
        SdCmd cmd = {};
        cmd.type = SDC_LOG_START;
        cmd.arg  = LOG_DEFAULT_INTERVAL_MS;
        if (len >= 5) memcpy(&cmd.arg, d + 1, 4);
        xQueueSend(sdCmdQueue, &cmd, 0);
        break;
      }
      case 0x11: {
        SdCmd cmd = {};
        cmd.type = SDC_LOG_STOP;
        xQueueSend(sdCmdQueue, &cmd, 0);
        break;
      }
      case 0x12:
        if (len >= 11) {
          int64_t epochMs;
          int16_t tzMin;
          memcpy(&epochMs, d + 1, 8);
          memcpy(&tzMin, d + 9, 2);
          setClock(epochMs, tzMin);
        }
        break;
      case 0x13:
        calRequest = true;   // die Messung selbst macht der ADC-Task
        break;
    }
  }
};

class ChannelsCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    if (subValue > 0) {
      BleJob job = { JOB_CHANNELS, connInfo.getConnHandle(), connInfo.getMTU() };
      xQueueSend(bleJobQueue, &job, 0);   // eigentliches Senden passiert in loop(), nicht hier im Callback
    }
  }
};

class HistoryCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    if (subValue > 0) {
      BleJob job = { JOB_HISTORY, connInfo.getConnHandle(), connInfo.getMTU() };
      xQueueSend(bleJobQueue, &job, 0);
    }
  }
};

// Befehlsformat (Client -> ESP32), erstes Byte = Typ:
//   1 = Marker setzen (Server vergibt Zeit+ID+Label)
//   2 = Marker loeschen (Bytes 1-2 = Marker-ID als uint16, little endian)
//   3 = Alle Marker loeschen
class MarkersCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = pChar->getValue();
    size_t len = val.length();
    if (len == 0) return;
    const uint8_t* d = val.data();

    uint32_t now = millis();
    portENTER_CRITICAL(&stateMux);
    if (d[0] == 1) {
      {
        if (markerCount >= MAX_MARKERS) {
          // Liste voll: aeltesten Marker aus der Anzeige nehmen, damit bei langen
          // Aufzeichnungen weiter Marker gesetzt (und in die CSV geschrieben) werden koennen
          for (uint8_t j = 0; j + 1 < markerCount; j++) markerList[j] = markerList[j + 1];
          markerCount--;
        }
        MarkerRecord& m = markerList[markerCount++];
        m.id   = nextMarkerId++;
        m.t_ms = now;
        // Label "M<id>" von Hand zusammensetzen (kein snprintf im kritischen Abschnitt)
        char digits[6];
        uint8_t nd = 0;
        uint16_t v = m.id;
        do { digits[nd++] = (char)('0' + v % 10); v /= 10; } while (v && nd < sizeof(digits));
        uint8_t pos = 0;
        m.label[pos++] = 'M';
        while (nd) m.label[pos++] = digits[--nd];
        m.label[pos] = 0;
        pendingLogMarker = m.id;   // landet auch in der CSV der Langzeitmessung
      }
    } else if (d[0] == 2 && len >= 3) {
      uint16_t delId = (uint16_t)(d[1] | (d[2] << 8));
      for (uint8_t i = 0; i < markerCount; i++) {
        if (markerList[i].id == delId) {
          for (uint8_t j = i; j + 1 < markerCount; j++) markerList[j] = markerList[j + 1];
          markerCount--;
          break;
        }
      }
    } else if (d[0] == 3) {
      markerCount = 0;
    }
    portEXIT_CRITICAL(&stateMux);
    markersPending = true;
  }

  void onSubscribe(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    if (subValue > 0) markersPending = true;   // neu verbundenes Geraet auf den aktuellen Stand bringen
  }
};

// Befehlsformat (Client -> ESP32), erstes Byte = Typ:
//   1 = Dateiliste anfordern
//   2 = Datei herunterladen (ab Byte 1: Dateiname)
//   3 = Datei loeschen      (ab Byte 1: Dateiname)
//   4 = laufenden Download abbrechen
class FilesCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = pChar->getValue();
    size_t len = val.length();
    if (len == 0) return;
    const uint8_t* d = val.data();

    SdCmd cmd = {};
    cmd.conn = connInfo.getConnHandle();
    cmd.mtu  = connInfo.getMTU();
    if (d[0] == 1) {
      cmd.type = SDC_LIST;
    } else if (d[0] == 2 || d[0] == 3) {
      cmd.type = (d[0] == 2) ? SDC_DOWNLOAD : SDC_DELETE;
      size_t n = len - 1;
      if (n > sizeof(cmd.name) - 1) n = sizeof(cmd.name) - 1;
      memcpy(cmd.name, d + 1, n);
      cmd.name[n] = 0;
    } else if (d[0] == 4) {
      fileCancel = true;
      return;
    } else {
      return;
    }
    xQueueSend(sdCmdQueue, &cmd, 0);
  }
};

// ====================================================================
//  Akku, Status, serielle Konsole (laufen in loop())
// ====================================================================
void batteryService() {
  static uint32_t last = 0;
  static float    filt = 0;
  static uint8_t  critCount = 0;
  if (millis() - last < 250) return;
  last = millis();

  float v = analogReadMilliVolts(PIN_VBAT_ADC) * 0.001f * BAT_DIVIDER * BAT_CAL;
  filt = (filt == 0) ? v : filt + 0.2f * (v - filt);
  batteryMv = (uint16_t)(filt * 1000.0f);

  bool valid = filt >= BAT_VALID_MIN_V;
  batLow = valid && filt < BAT_WARN_V;
  if (valid && filt < BAT_CRIT_V) {
    if (critCount < 255) critCount++;
  } else {
    critCount = 0;
  }
  // 3 s durchgehend unter der Schwelle. Bleibt bis zum Neustart gesetzt -
  // ein Akkuwechsel ist bei diesem Geraet immer ein Neustart.
  if (critCount >= 12 && !batCritical) {
    batCritical = true;
    Serial.println("WARNUNG: Akku leer - Langzeitmessung wird beendet.");
  }
}

void statusService() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  StatusPacket st;
  fillStatus(st);
  pStatusChar->setValue((const uint8_t*)&st, sizeof(st));
  if (connCount > 0) pStatusChar->notify();
}

// Ergebnis eines Nullabgleichs dauerhaft speichern; calSeq meldet der Weboberflaeche "fertig".
void calibrationService() {
  if (!calSavePending) return;
  calSavePending = false;
  float z[4];
  for (uint8_t c = 0; c < 4; c++) z[c] = aiZeroV[c];
  if (calMask) prefs.putBytes("aiZero", z, sizeof(z));
  calSeq = calSeq + 1;
  Serial.printf("Nullabgleich: Maske 0x%X, Korrektur %.1f / %.1f / %.1f / %.1f mV\n", calMask,
                z[0] * 1000.0, z[1] * 1000.0, z[2] * 1000.0, z[3] * 1000.0);
}

void serialService() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 's' || c == 'S' || c == '?') printStatus();
  }
}

// ====================================================================
//  Setup / Loop
// ====================================================================
void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);   // ohne angeschlossenen PC nicht auf die USB-Ausgabe warten
#endif

  for (uint8_t i = 0; i < 4; i++) pinMode(PIN_DI[i], INPUT);   // ISO1211 treibt aktiv HIGH/LOW
  pinMode(PIN_SD_CD, INPUT_PULLUP);                            // zusaetzlich zu R110 auf der Platine
  analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);             // Messbereich bis ca. 3,1 V am Pin

  prefs.begin("messgeraet", false);
  if (prefs.getBytesLength("aiZero") == 4 * sizeof(float)) {   // gespeicherten Nullabgleich laden
    float z[4];
    prefs.getBytes("aiZero", z, sizeof(z));
    for (uint8_t c = 0; c < 4; c++) if (fabsf(z[c]) <= AI_ZERO_MAX_V) aiZeroV[c] = z[c];
  }

  logQueue    = xQueueCreate(256, sizeof(LogItem));   // ca. 10 s Reserve bei 25 Zeilen/s
  sdCmdQueue  = xQueueCreate(6, sizeof(SdCmd));
  bleJobQueue = xQueueCreate(8, sizeof(BleJob));

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);   // I2C Fast Mode
  Wire.setTimeOut(20);                            // haengt der Bus, abbrechen statt lange warten

  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3);

  // ---------- BLE ----------
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(9);     // +9 dBm, mehr Reichweite im Schaltschrank/Werkstattumfeld
  NimBLEDevice::setMTU(247);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pChannelsChar = pService->createCharacteristic(CHANNELS_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  pChannelsChar->setCallbacks(new ChannelsCallbacks());
  buildChannelsJson();

  pSampleChar = pService->createCharacteristic(SAMPLE_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  pControlChar = pService->createCharacteristic(CONTROL_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  pControlChar->setCallbacks(new ControlCallbacks());

  pHistoryChar = pService->createCharacteristic(HISTORY_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  pHistoryChar->setCallbacks(new HistoryCallbacks());

  pMarkersChar = pService->createCharacteristic(MARKERS_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pMarkersChar->setCallbacks(new MarkersCallbacks());

  pStatusChar = pService->createCharacteristic(STATUS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pFilesChar = pService->createCharacteristic(FILES_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pFilesChar->setCallbacks(new FilesCallbacks());

  pService->start();   // ab NimBLE-Arduino 2.5 nicht mehr noetig (dort nur eine Compiler-Warnung)

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->enableScanResponse(true);      // Geraetename passt neben der 128-Bit-UUID nur in die Scan-Antwort
  pAdvertising->setName(BLE_DEVICE_NAME);
  pAdvertising->start();

  // ---------- Tasks ----------
  // ADC und Messtakt auf Kern 1 (wie loop()), SD-Karte auf Kern 0.
  xTaskCreatePinnedToCore(adcTask,    "adc",    6144,  nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(sampleTask, "sample", 6144,  nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(sdTask,     "sd",     10240, nullptr, 2, nullptr, 0);

  Serial.printf("Messgeraet %s gestartet, BLE-Werbung laeuft ('s' = Status)\n", FW_VERSION);
}

void loop() {
  BleJob job;
  if (xQueueReceive(bleJobQueue, &job, pdMS_TO_TICKS(20)) == pdTRUE) {
    delay(50);   // Verbindung kurz stabilisieren lassen, bevor der Burst losgeht
    if (connAlive(job.conn)) {
      if (job.type == JOB_CHANNELS) sendChannels(job.conn, job.mtu);
      else                          sendHistory(job.conn, job.mtu);
    }
  }

  if (markersPending) {
    markersPending = false;
    broadcastMarkers();
  }

  batteryService();
  calibrationService();
  statusService();
  serialService();
}
