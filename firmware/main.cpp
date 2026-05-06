#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <SD.h>
#include <FS.h>
#include <WiFi.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

// =====================
// Wi-Fi / NTP
// =====================
const char *WIFI_SSID = "Visitantes";
const char *NTP_SERVER_1 = "pool.ntp.org";
const char *NTP_SERVER_2 = "time.nist.gov";
const char *NTP_SERVER_3 = "time.google.com";

// Curitiba / America/Sao_Paulo
const char *TZ_INFO = "<-03>3";

bool wifiOK = false;
bool timeOK = false;

// =====================
// MCP2515 (VSPI)
// =====================
#define CAN_CS   5
#define CAN_INT  4
#define CAN_SCK  18
#define CAN_MISO 19
#define CAN_MOSI 23

// =====================
// SD Card (HSPI)
// =====================
#define SD_CS    33
#define SD_SCK   14
#define SD_MISO  27
#define SD_MOSI  13

MCP_CAN CAN(CAN_CS);
SPIClass sdSPI(HSPI);

bool sdOK = false;

// =====================
// Arquivos
// =====================
const char *INDEX_FILE = "/events_index.csv";

// =====================
// PGNs de interesse
// =====================
const unsigned long EEC1_PGN = 61444UL; // 0xF004
const unsigned long DM1_PGN  = 65226UL; // 0xFECA

// =====================
// Parâmetros
// =====================
// Janela final proposta para o TCC:
// - 5 min antes da falha  -> PRE
// - duração da falha      -> ACTIVE, controlada pelo DM1 recebido
// - 5 min depois da falha -> POST
//
// Observação: o pré e o pós-evento são amostrados em 250 ms para manter o uso
// de RAM do ESP32 em uma faixa segura. Durante o evento ativo, a amostragem
// continua mais rápida, em 20 ms.
const unsigned long SAMPLE_INTERVAL_IDLE_MS   = 250;      // PRE e POST
const unsigned long SAMPLE_INTERVAL_ACTIVE_MS = 20;       // ACTIVE
const unsigned long PRE_EVENT_MS              = 300000UL; // 5 min
const unsigned long POST_EVENT_MS             = 300000UL; // 5 min
const unsigned long STARTUP_GRACE_MS          = 5000UL;

const unsigned long DM1_ACTIVE_HOLD_MS        = 1500UL;

// Capacidade calculada para armazenar 5 min de PRE + margem de segurança.
// Com 250 ms: 300000 / 250 = 1200 snapshots.
const int PREBUFFER_CAPACITY = (PRE_EVENT_MS / SAMPLE_INTERVAL_IDLE_MS) + 100;
const unsigned long STATUS_PRINT_INTERVAL_MS = 1000UL;

// =====================
// Buffer de escrita no SD
// =====================
File currentEventFile;
bool currentEventFileOpen = false;
String eventWriteBuffer = "";
unsigned long lastEventFlushMs = 0;
const size_t EVENT_BUFFER_FLUSH_BYTES = 4096;
const unsigned long EVENT_BUFFER_FLUSH_MS = 1000;

// ======================================================
// Estruturas
// ======================================================
struct EEC1State {
  bool rpmValid;
  float rpm;

  bool drvDemTqValid;
  int16_t drvDemTq;

  bool actTqValid;
  int16_t actTq;


  uint8_t sa;
};

struct DM1State {
  bool valid;
  uint8_t sa;
  uint8_t lamp1;
  uint8_t lamp2;
  uint32_t spn;
  uint8_t fmi;
  uint8_t cm;
};

struct EventDTCInfo {
  bool valid;
  uint8_t sa;
  uint32_t spn;
  uint8_t fmi;
  uint8_t cm;
};

struct Snapshot {
  unsigned long ms;

  bool inStartupGrace;

  bool rpmValid;
  float rpm;

  bool drvDemTqValid;
  int16_t drvDemTq;

  bool actTqValid;
  int16_t actTq;


  uint8_t eec1Sa;

  bool dm1Active;
  bool dm1Valid;
  uint8_t dm1Sa;
  uint32_t dm1Spn;
  uint8_t dm1Fmi;
  uint8_t dm1Cm;

  int16_t torqueGapValue;

  bool triggerFaultActive;

  bool rtcValid;
  uint64_t epochMs;
};

// ======================================================
// Estado global
// ======================================================
unsigned long bootMs = 0;
unsigned long lastSnapshotMs = 0;
unsigned long lastStatusPrintMs = 0;

unsigned long lastEEC1Ms = 0;
unsigned long lastDM1Ms = 0;

EEC1State currentEEC1 = {false, 0.0f, false, 0, false, 0, 0};
DM1State currentDM1 = {false, 0, 0, 0, 0, 0, 0};
EventDTCInfo currentEventDTC = {false, 0, 0, 0, 0};

uint64_t currentEventStartEpochMs = 0;
bool currentEventStartRtcValid = false;

// ======================================================
// Buffer circular dos últimos 5 min de PRE
// ======================================================
Snapshot preBuffer[PREBUFFER_CAPACITY];
int preBufHead = 0;
int preBufCount = 0;

// ======================================================
// Estado de evento
// ======================================================
bool eventOpen = false;
bool postWindowStarted = false;

unsigned long postWindowEndMs = 0;
unsigned long currentEventStartMs = 0;

unsigned int nextEventId = 1;
unsigned int currentEventId = 0;

String currentEventPath = "";

// ======================================================
// Helpers de texto / tempo
// ======================================================
String formatUptime(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000;
  unsigned long hours = totalSeconds / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;
  unsigned long millisPart = ms % 1000;

  char buffer[20];
  sprintf(buffer, "%02lu:%02lu:%02lu.%03lu", hours, minutes, seconds, millisPart);
  return String(buffer);
}

void logLine(const String &msg) {
  Serial.print("[");
  Serial.print(formatUptime(millis()));
  Serial.print("] ");
  Serial.println(msg);
}

String sanitizeText(String s) {
  s.replace("\n", " ");
  s.replace("\r", " ");
  return s;
}

String csvQuote(String s) {
  s = sanitizeText(s);
  s.replace("\"", "\"\"");
  return "\"" + s + "\"";
}

String maybeFloat(bool valid, float value, int decimals = 2) {
  if (!valid) return "";
  return String(value, decimals);
}

String maybeInt(bool valid, int value) {
  if (!valid) return "";
  return String(value);
}

bool getCurrentEpochMs(uint64_t &epochMs) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  if (tv.tv_sec < 1700000000) {
    return false;
  }

  epochMs = ((uint64_t)tv.tv_sec * 1000ULL) + ((uint64_t)tv.tv_usec / 1000ULL);
  return true;
}

String formatDateFromEpochMs(uint64_t epochMs, bool valid) {
  if (!valid) return "";

  time_t sec = (time_t)(epochMs / 1000ULL);
  struct tm timeinfo;
  localtime_r(&sec, &timeinfo);

  char buffer[16];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d", &timeinfo);
  return String(buffer);
}

String formatTimeFromEpochMs(uint64_t epochMs, bool valid) {
  if (!valid) return "";

  time_t sec = (time_t)(epochMs / 1000ULL);
  struct tm timeinfo;
  localtime_r(&sec, &timeinfo);

  char buffer[16];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", &timeinfo);

  char out[24];
  snprintf(out, sizeof(out), "%s.%03llu", buffer, (unsigned long long)(epochMs % 1000ULL));
  return String(out);
}

// ======================================================
// Wi-Fi / NTP
// ======================================================
bool connectWiFiOpen(const char *ssid, unsigned long timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, NULL);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(250);
  }

  return WiFi.status() == WL_CONNECTED;
}

bool syncTimeNTP(unsigned long timeoutMs) {
  setenv("TZ", TZ_INFO, 1);
  tzset();

  configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);

  unsigned long start = millis();
  time_t now = time(nullptr);

  while (now < 1700000000 && (millis() - start) < timeoutMs) {
    delay(250);
    now = time(nullptr);
  }

  return now >= 1700000000;
}

void shutdownWiFi() {
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_OFF);
  logLine("Wi-Fi desligado apos sincronizacao");
}

// ======================================================
// SD / Arquivo
// ======================================================
void appendLineToFile(const char *path, const String &line) {
  if (!sdOK) return;

  File f = SD.open(path, FILE_APPEND);
  if (!f) {
    logLine("Erro ao abrir " + String(path));
    return;
  }

  f.println(line);
  f.close();
}

void ensureFileWithHeader(const char *path, const String &header) {
  if (!sdOK) return;

  if (!SD.exists(path)) {
    File f = SD.open(path, FILE_WRITE);
    if (f) {
      f.println(header);
      f.close();
      logLine("Arquivo criado: " + String(path));
    } else {
      logLine("Erro ao criar " + String(path));
    }
  } else {
    logLine("Arquivo ja existe: " + String(path));
  }
}

bool mountSDOnce(uint32_t freqHz) {
  SD.end();
  sdSPI.end();

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  delay(100);

  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  delay(200);

  return SD.begin(SD_CS, sdSPI, freqHz);
}

bool initSDRobust() {
  logLine("Preparando SD...");
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  delay(800);

  uint32_t freqs[] = {400000, 250000, 100000};

  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
    logLine("Tentando SD em " + String(freqs[i]) + " Hz...");

    for (int attempt = 1; attempt <= 3; attempt++) {
      logLine("Tentativa " + String(attempt));

      if (mountSDOnce(freqs[i])) {
        logLine("SD montado com sucesso!");
        return true;
      }

      delay(250);
    }

    logLine("Falhou nessa frequencia.");
  }

  return false;
}

void flushEventFile(bool force) {
  if (!currentEventFileOpen) return;
  if (eventWriteBuffer.length() == 0) return;

  if (!force) {
    if (eventWriteBuffer.length() < EVENT_BUFFER_FLUSH_BYTES &&
        (millis() - lastEventFlushMs) < EVENT_BUFFER_FLUSH_MS) {
      return;
    }
  }

  currentEventFile.print(eventWriteBuffer);
  currentEventFile.flush();
  eventWriteBuffer = "";
  lastEventFlushMs = millis();
}

void appendEventBuffered(const String &line) {
  if (!currentEventFileOpen) return;

  eventWriteBuffer += line;
  eventWriteBuffer += "\n";

  if (eventWriteBuffer.length() >= EVENT_BUFFER_FLUSH_BYTES) {
    flushEventFile(false);
  }
}

void closeEventFile() {
  if (!currentEventFileOpen) return;

  flushEventFile(true);
  currentEventFile.close();
  currentEventFileOpen = false;
}

// ======================================================
// J1939 helpers
// ======================================================
unsigned long extractPGN(unsigned long id) {
  uint8_t pf = (id >> 16) & 0xFF;
  unsigned long pgn = (id >> 8) & 0x3FFFF;

  if (pf < 240) {
    pgn &= 0x3FF00;
  }

  return pgn;
}

// ======================================================
// EEC1 decode
// ======================================================
// EEC1 de bancada:
// byte 0 -> drv_dem_tq
// byte 1 -> act_tq
// byte 3-4 -> rpm
// bytes 2, 5, 6 e 7 -> nao utilizados nesta versao
bool decodeEEC1_RPM(const byte *buf, byte len, float &rpm) {
  if (len < 5) return false;

  uint16_t raw = ((uint16_t)buf[4] << 8) | buf[3];
  if (raw == 0xFFFF) return false;

  rpm = raw * 0.125f;
  return true;
}

bool decodeEEC1_PercentTorque(uint8_t raw, int16_t &pct) {
  if (raw == 0xFF) return false;
  pct = (int16_t)raw - 125;
  return true;
}


// ======================================================
// DM1 decode (single-frame, primeiro DTC)
// ======================================================
bool parseDM1SingleFrame(const byte *buf, byte len, unsigned long id, DM1State &out) {
  if (len < 6) return false;

  out.valid = false;
  out.sa = id & 0xFF;
  out.lamp1 = buf[0];
  out.lamp2 = buf[1];

  if (buf[2] == 0xFF && buf[3] == 0xFF && buf[4] == 0xFF && buf[5] == 0xFF) {
    return false;
  }

  uint32_t spn = 0;
  spn |= (uint32_t)buf[2];
  spn |= ((uint32_t)buf[3] << 8);
  spn |= ((uint32_t)(buf[4] & 0xE0) << 11);

  uint8_t fmi = buf[4] & 0x1F;
  uint8_t cm  = (buf[5] >> 7) & 0x01;

  out.spn = spn;
  out.fmi = fmi;
  out.cm = cm;
  out.valid = true;

  return true;
}

// ======================================================
// Nome amigável de SPN / FMI
// ======================================================
const char* spnName(uint32_t spn) {
  switch (spn) {
    case 190:   return "Engine_Speed";
    case 512:   return "Driver_Demand_Engine_Percent_Torque";
    case 513:   return "Actual_Engine_Percent_Torque";
    default:    return "Unknown_SPN";
  }
}

const char* fmiName(uint8_t fmi) {
  switch (fmi) {
    case 0:  return "High_Most_Severe";
    case 1:  return "Low_Most_Severe";
    case 2:  return "Data_Erratic";
    case 3:  return "Voltage_Above_Normal";
    case 4:  return "Voltage_Below_Normal";
    case 5:  return "Current_Below_Normal";
    case 6:  return "Current_Above_Normal";
    case 7:  return "Mechanical_Not_Responding";
    case 8:  return "Abnormal_Frequency";
    case 9:  return "Abnormal_Update_Rate";
    case 10: return "Abnormal_Rate_Of_Change";
    case 12: return "Bad_Intelligent_Device";
    case 13: return "Out_Of_Calibration";
    case 31: return "Condition_Exists";
    default: return "Unknown_FMI";
  }
}

// ======================================================
// Buffer circular
// ======================================================
int oldestBufferIndex() {
  return (preBufHead - preBufCount + PREBUFFER_CAPACITY) % PREBUFFER_CAPACITY;
}

void pruneBufferByTime(unsigned long nowMs) {
  while (preBufCount > 0) {
    int idx = oldestBufferIndex();
    if ((nowMs - preBuffer[idx].ms) > PRE_EVENT_MS) {
      preBufCount--;
    } else {
      break;
    }
  }
}

void pushSnapshotToBuffer(const Snapshot &snap) {
  preBuffer[preBufHead] = snap;
  preBufHead = (preBufHead + 1) % PREBUFFER_CAPACITY;

  if (preBufCount < PREBUFFER_CAPACITY) {
    preBufCount++;
  }

  pruneBufferByTime(snap.ms);
}

// ======================================================
// Arquivos por evento
// ======================================================
String eventPathForId(unsigned int eventId) {
  char name[20];
  sprintf(name, "/event_%04u.csv", eventId);
  return String(name);
}

void initNextEventId() {
  if (!sdOK) return;

  while (SD.exists(eventPathForId(nextEventId).c_str())) {
    nextEventId++;
  }
}

void latchEventDTC(const Snapshot &snap) {
  currentEventDTC.valid = snap.dm1Valid;
  currentEventDTC.sa = snap.dm1Sa;
  currentEventDTC.spn = snap.dm1Spn;
  currentEventDTC.fmi = snap.dm1Fmi;
  currentEventDTC.cm = snap.dm1Cm;

  currentEventStartEpochMs = snap.epochMs;
  currentEventStartRtcValid = snap.rtcValid;
}

void appendSnapshotToCurrentEvent(const Snapshot &snap, const String &segment) {
  if (!sdOK || !eventOpen || !currentEventFileOpen) return;

  bool showDTC = (segment == "ACTIVE");

  String line = "";
  line += csvQuote(formatDateFromEpochMs(snap.epochMs, snap.rtcValid)) + ";";
  line += csvQuote(formatTimeFromEpochMs(snap.epochMs, snap.rtcValid)) + ";";
  line += csvQuote(segment) + ";";
  line += String(snap.eec1Sa) + ";";
  line += maybeFloat(snap.rpmValid, snap.rpm, 2) + ";";
  line += maybeInt(snap.drvDemTqValid, snap.drvDemTq) + ";";
  line += maybeInt(snap.actTqValid, snap.actTq) + ";";
  line += maybeInt((snap.drvDemTqValid && snap.actTqValid), snap.torqueGapValue) + ";";
  line += String(snap.dm1Active ? 1 : 0) + ";";
  line += (showDTC && currentEventDTC.valid ? String(currentEventDTC.sa) : "") + ";";
  line += (showDTC && currentEventDTC.valid ? String(currentEventDTC.spn) : "") + ";";
  line += csvQuote(showDTC && currentEventDTC.valid ? String(spnName(currentEventDTC.spn)) : "") + ";";
  line += (showDTC && currentEventDTC.valid ? String(currentEventDTC.fmi) : "") + ";";
  line += csvQuote(showDTC && currentEventDTC.valid ? String(fmiName(currentEventDTC.fmi)) : "") + ";";
  line += (showDTC && currentEventDTC.valid ? String(currentEventDTC.cm) : "");

  appendEventBuffered(line);
}

void startNewEvent(const Snapshot &triggerSnap) {
  if (!sdOK) {
    logLine("[EVENT] SD indisponivel, evento nao sera salvo");
    return;
  }

  currentEventId = nextEventId++;
  currentEventStartMs = triggerSnap.ms;
  currentEventPath = eventPathForId(currentEventId);

  latchEventDTC(triggerSnap);

  currentEventFile = SD.open(currentEventPath.c_str(), FILE_WRITE);
  if (!currentEventFile) {
    logLine("Erro ao criar " + currentEventPath);
    return;
  }

  currentEventFileOpen = true;
  eventWriteBuffer = "";
  eventWriteBuffer.reserve(8192);
  lastEventFlushMs = millis();

  currentEventFile.println("local_date;local_time;segment;eec1_sa;rpm;drv_dem_tq;act_tq;torque_gap;dm1_active;dtc_sa;dtc_spn;dtc_param;dtc_fmi;dtc_fmi_name;dtc_cm");
  currentEventFile.flush();

  eventOpen = true;
  postWindowStarted = false;
  postWindowEndMs = 0;

  int startIdx = oldestBufferIndex();
  for (int i = 0; i < preBufCount; i++) {
    int idx = (startIdx + i) % PREBUFFER_CAPACITY;
    Snapshot snap = preBuffer[idx];

    String seg = (snap.ms < currentEventStartMs) ? "PRE" : "ACTIVE";
    appendSnapshotToCurrentEvent(snap, seg);
  }

  flushEventFile(true);

  String dtcSummary = "";
  if (currentEventDTC.valid) {
    dtcSummary = "sa=" + String(currentEventDTC.sa) +
                 "|spn=" + String(currentEventDTC.spn) +
                 "|param=" + String(spnName(currentEventDTC.spn)) +
                 "|fmi=" + String(currentEventDTC.fmi) +
                 "|fmi_name=" + String(fmiName(currentEventDTC.fmi)) +
                 "|cm=" + String(currentEventDTC.cm);
  } else {
    dtcSummary = "none";
  }

  logLine("[EVENT START] #" + String(currentEventId) +
          " | arquivo=" + currentEventPath +
          " | dtc=" + dtcSummary);
}

void finalizeCurrentEvent(const Snapshot &endSnap) {
  if (!eventOpen || !sdOK) return;

  flushEventFile(true);
  closeEventFile();

  String summary = "";
  summary += String(currentEventId) + ";";
  summary += csvQuote(currentEventPath) + ";";
  summary += csvQuote(formatDateFromEpochMs(currentEventStartEpochMs, currentEventStartRtcValid)) + ";";
  summary += csvQuote(formatTimeFromEpochMs(currentEventStartEpochMs, currentEventStartRtcValid)) + ";";
  summary += csvQuote(formatDateFromEpochMs(endSnap.epochMs, endSnap.rtcValid)) + ";";
  summary += csvQuote(formatTimeFromEpochMs(endSnap.epochMs, endSnap.rtcValid)) + ";";
  summary += (currentEventDTC.valid ? String(currentEventDTC.sa) : "") + ";";
  summary += (currentEventDTC.valid ? String(currentEventDTC.spn) : "") + ";";
  summary += csvQuote(currentEventDTC.valid ? String(spnName(currentEventDTC.spn)) : "") + ";";
  summary += (currentEventDTC.valid ? String(currentEventDTC.fmi) : "") + ";";
  summary += csvQuote(currentEventDTC.valid ? String(fmiName(currentEventDTC.fmi)) : "") + ";";
  summary += (currentEventDTC.valid ? String(currentEventDTC.cm) : "");

  appendLineToFile(INDEX_FILE, summary);

  logLine("[EVENT END] #" + String(currentEventId) +
          " | arquivo=" + currentEventPath +
          " | inicio=" + formatTimeFromEpochMs(currentEventStartEpochMs, currentEventStartRtcValid) +
          " | fim=" + formatTimeFromEpochMs(endSnap.epochMs, endSnap.rtcValid));

  eventOpen = false;
  postWindowStarted = false;
  postWindowEndMs = 0;
  currentEventStartMs = 0;
  currentEventId = 0;
  currentEventPath = "";
  currentEventDTC = {false, 0, 0, 0, 0};
  currentEventStartEpochMs = 0;
  currentEventStartRtcValid = false;
}

// ======================================================
// Snapshot periódico
// ======================================================
Snapshot buildSnapshot(unsigned long now) {
  Snapshot snap = {};

  snap.ms = now;
  snap.inStartupGrace = ((now - bootMs) < STARTUP_GRACE_MS);

  snap.rpmValid = currentEEC1.rpmValid;
  snap.rpm = currentEEC1.rpm;

  snap.drvDemTqValid = currentEEC1.drvDemTqValid;
  snap.drvDemTq = currentEEC1.drvDemTq;

  snap.actTqValid = currentEEC1.actTqValid;
  snap.actTq = currentEEC1.actTq;


  snap.eec1Sa = currentEEC1.sa;

  snap.dm1Active = currentDM1.valid && ((now - lastDM1Ms) <= DM1_ACTIVE_HOLD_MS);
  snap.dm1Valid = currentDM1.valid;
  snap.dm1Sa = currentDM1.sa;
  snap.dm1Spn = currentDM1.spn;
  snap.dm1Fmi = currentDM1.fmi;
  snap.dm1Cm = currentDM1.cm;

  snap.torqueGapValue = 0;
  if (snap.drvDemTqValid && snap.actTqValid) {
    snap.torqueGapValue = snap.drvDemTq - snap.actTq;
  }

  snap.triggerFaultActive = !snap.inStartupGrace && snap.dm1Active;

  snap.rtcValid = getCurrentEpochMs(snap.epochMs);

  return snap;
}

unsigned long currentSampleIntervalMs() {
  if (eventOpen && !postWindowStarted) {
    return SAMPLE_INTERVAL_ACTIVE_MS; // só ACTIVE mais rápido
  }
  return SAMPLE_INTERVAL_IDLE_MS;
}

void handleSnapshot(const Snapshot &snap) {
  pushSnapshotToBuffer(snap);

  if (!sdOK) return;

  if (!eventOpen) {
    if (snap.triggerFaultActive) {
      startNewEvent(snap);
    }
    return;
  }

  if (snap.triggerFaultActive) {
    postWindowStarted = false;
    postWindowEndMs = 0;
    appendSnapshotToCurrentEvent(snap, "ACTIVE");
  } else {
    if (!postWindowStarted) {
      postWindowStarted = true;
      postWindowEndMs = snap.ms + POST_EVENT_MS;
    }

    appendSnapshotToCurrentEvent(snap, "POST");

    if (snap.ms >= postWindowEndMs) {
      finalizeCurrentEvent(snap);
    }
  }
}

// ======================================================
// Processamento CAN
// ======================================================
void processEEC1(unsigned long id, byte len, const byte *buf, unsigned long now) {
  float rpm = 0.0f;
  int16_t drvDemTq = 0;
  int16_t actTq = 0;
  bool rpmOk = decodeEEC1_RPM(buf, len, rpm);
  bool drvOk = (len >= 1) ? decodeEEC1_PercentTorque(buf[0], drvDemTq) : false;
  bool actOk = (len >= 2) ? decodeEEC1_PercentTorque(buf[1], actTq) : false;

  if (rpmOk) {
    lastEEC1Ms = now;
  }

  currentEEC1.rpmValid = rpmOk;
  currentEEC1.rpm = rpm;
  currentEEC1.drvDemTqValid = drvOk;
  currentEEC1.drvDemTq = drvDemTq;
  currentEEC1.actTqValid = actOk;
  currentEEC1.actTq = actTq;
  currentEEC1.sa = id & 0xFF;
}

void processDM1(unsigned long id, byte len, const byte *buf, unsigned long now) {
  DM1State parsed = {};
  if (parseDM1SingleFrame(buf, len, id, parsed)) {
    currentDM1 = parsed;
    lastDM1Ms = now;
  }
}

void processCAN() {
  unsigned long id = 0;
  byte ext = 0;
  byte len = 0;
  byte buf[8];

  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    if (CAN.readMsgBuf(&id, &ext, &len, buf) != CAN_OK) {
      break;
    }

    if (!ext) continue;

    unsigned long pgn = extractPGN(id);
    unsigned long now = millis();

    if (pgn == EEC1_PGN) {
      processEEC1(id, len, buf, now);
    } else if (pgn == DM1_PGN) {
      processDM1(id, len, buf, now);
    }
  }
}

// ======================================================
// Status serial
// ======================================================
String buildStatusSignals() {
  String s = "";
  if (currentEEC1.rpmValid) s += "rpm=" + String(currentEEC1.rpm, 2);
  if (currentEEC1.drvDemTqValid) {
    if (s.length()) s += "|";
    s += "drv_dem_tq=" + String(currentEEC1.drvDemTq);
  }
  if (currentEEC1.actTqValid) {
    if (s.length()) s += "|";
    s += "act_tq=" + String(currentEEC1.actTq);
  }
  if (s.length() == 0) s = "none";
  return s;
}

void printStatusIfNeeded(unsigned long now) {
  if ((now - lastStatusPrintMs) < STATUS_PRINT_INTERVAL_MS) return;

  Snapshot snap = buildSnapshot(now);

  String msg = "[STATUS] ";
  msg += "wifi=" + String(wifiOK ? "OK" : "OFF");
  msg += "|time=" + String(timeOK ? "OK" : "FAIL");
  msg += " | signals=" + buildStatusSignals();
  msg += " | dtc=";
  if (snap.dm1Active && snap.dm1Valid) {
    msg += "sa=" + String(snap.dm1Sa) +
           "|spn=" + String(snap.dm1Spn) +
           "|param=" + String(spnName(snap.dm1Spn)) +
           "|fmi=" + String(snap.dm1Fmi);
  } else {
    msg += "none";
  }
  msg += " | trigger=" + String(snap.triggerFaultActive ? "ON" : "OFF");
  msg += " | event=" + String(eventOpen ? "ON" : "OFF");
  msg += " | sd=" + String(sdOK ? "OK" : "FAIL");

  logLine(msg);

  lastStatusPrintMs = now;
}

// ======================================================
// Setup
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(CAN_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(CAN_INT, INPUT);

  digitalWrite(CAN_CS, HIGH);
  digitalWrite(SD_CS, HIGH);

  bootMs = millis();

  Serial.println();
  logLine("=== MONITOR J1939 + WIFI/NTP + LOG 5 MIN PRE/POST ===");

  logLine("Conectando Wi-Fi em rede aberta...");
  wifiOK = connectWiFiOpen(WIFI_SSID, 12000);
  if (wifiOK) {
    logLine("Wi-Fi OK | IP=" + WiFi.localIP().toString());
    timeOK = syncTimeNTP(15000);
    if (timeOK) {
      uint64_t epochMs = 0;
      bool valid = getCurrentEpochMs(epochMs);
      logLine(String("Hora sincronizada: ") +
              formatDateFromEpochMs(epochMs, valid) + " " +
              formatTimeFromEpochMs(epochMs, valid));
    } else {
      logLine("Wi-Fi conectado, mas NTP nao sincronizou");
    }
    shutdownWiFi();
    wifiOK = false; // agora está desligado de propósito
  } else {
    logLine("Falha ao conectar no Wi-Fi");
  }

  logLine("Iniciando SD...");
  sdOK = initSDRobust();

  if (sdOK) {
    logLine("SD OK!");
    ensureFileWithHeader(
      INDEX_FILE,
      "event_id;file;start_date;start_time;end_date;end_time;dtc_sa;dtc_spn;dtc_param;dtc_fmi;dtc_fmi_name;dtc_cm"
    );
    initNextEventId();
  } else {
    logLine("Erro no cartao SD");
  }

  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);

  logLine("Iniciando MCP2515...");
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    logLine("MCP2515 OK!");
    CAN.setMode(MCP_NORMAL);
    logLine("CAN pronto.");
  } else {
    logLine("Erro no MCP2515");
    while (1);
  }

  lastSnapshotMs = millis();
  lastStatusPrintMs = millis();

  logLine("Startup grace ativo por " + String(STARTUP_GRACE_MS) + " ms");
  logLine("ESCUTANDO CAN/J1939...");
}

// ======================================================
// Loop
// ======================================================
void loop() {
  processCAN();

  unsigned long now = millis();
  unsigned long sampleInterval = currentSampleIntervalMs();

  if ((now - lastSnapshotMs) >= sampleInterval) {
    lastSnapshotMs = now;

    Snapshot snap = buildSnapshot(now);
    handleSnapshot(snap);
  }

  flushEventFile(false);
  printStatusIfNeeded(now);
}