/*************************************************************************
 * AtomS3_Gyroflow_Logger.ino  – SD ラッパ関数版
 *************************************************************************/
 #include <SPI.h>
 #include <SdFat.h>
 #include <Wire.h>
 #include "M5AtomS3.h"
 #include <math.h>
 
 /* ---------- SD (Atomic GPS Base) ---------- */
 constexpr int SD_SCK  = 7;
 constexpr int SD_MOSI = 6;
 constexpr int SD_MISO = 8;
 constexpr int SD_CS   = 1;
 SdFs      sd;             // ★ SdFs = FAT16/32/exFAT 自動判別
 FsFile logFile;
 bool sd_dirty = false;
 uint32_t last_sync = 0;
 const uint32_t SYNC_INTERVAL_MS = 10000;  // 10 s ごと

 /* ---------- IMU / Timer ---------- */
 constexpr uint32_t SAMPLE_US = 1000;          // 1 kHz
 volatile bool sample_flag    = false;
 hw_timer_t* timer            = nullptr;
 portMUX_TYPE timerMux        = portMUX_INITIALIZER_UNLOCKED;
 
 /* ---------- Logging ---------- */
 String    currentFileName = "";
 bool      logging_active = false;
 
 /* ---------- LCD モニタ ---------- */
 float  disp_ax = 0, disp_ay = 0, disp_az = 0, disp_mag = 0;
 uint32_t last_lcd_ms = 0;
 
 /* ------------------------------------------------------------------ */
/*                 SdFat 用のシンプルラッパ（sd を直参照）              */
/* ------------------------------------------------------------------ */
bool existsFile(const char* path) {
  return sd.exists(path);
}

FsFile openFileWrite(const char* path) {
  // 追記モードで開き、なければ作成
  FsFile f = sd.open(path, O_WRITE | O_CREAT | O_APPEND);
  if (f) {                      // ★ 成功
    return f;
  }
  /*-------------------- error branch ----------------*/
  Serial.printf("[SD] open failed: %s\n", path);
  sd.printSdError(&Serial);                       // 人間可読メッセージ
  uint8_t ec = sd.card()->errorCode();
  uint8_t ed = sd.card()->errorData();
  Serial.printf("errorCode: 0x%02X  errorData: 0x%02X\n", ec, ed);

  /*--- 典型的な失敗コードに応じた対処例 -------------*/
  if (ec == SD_CARD_ERROR_ACMD41 && ed == 0xFF) {
    Serial.println("[HINT] カード初期化に失敗。配線か電源を確認してください。");
  }
  else if (ec == SD_CARD_ERROR_CMD17 || ec == SD_CARD_ERROR_CMD24) {
    Serial.println("[HINT] 書込み中にタイムアウト。クロックを下げるかカード交換を検討。");
  }
  else if (ec == 0 && ed == 0) {
    Serial.println("[HINT] パス名が長すぎる／ディレクトリが存在しない可能性。");
  }

  /* 失敗のまま空ハンドルを返す（呼び出し側で再判定） */
  return f;
}

 /* ---------- ログ完了後にファイルを検証 ---------- */
 void verifyLogFile() {
  if (currentFileName.isEmpty()) return;
  if (!existsFile(currentFileName.c_str())) {
    Serial.printf("[VERIFY] %s NOT FOUND\n", currentFileName.c_str());
    return;
  }
  FsFile f = sd.open(currentFileName.c_str(), O_READ);
  size_t sz = f.size();
  f.close();
  Serial.printf("[VERIFY] %s OK (%u bytes)\n",
                currentFileName.c_str(), (unsigned)sz);
}
 
 /* -------------------------------------------------------------------- */
 /*                              ISR (1 kHz)                              */
 /* -------------------------------------------------------------------- */
 void IRAM_ATTR onTimer()
 {
   portENTER_CRITICAL_ISR(&timerMux);
   sample_flag = true;
   portEXIT_CRITICAL_ISR(&timerMux);
 }
 
 /* ------------------------- Gyroflow ヘッダ --------------------------- */
 void writeGcsvHeader(FsFile &f)
 {
   f.println("GYROFLOW IMU LOG");
   f.println("version,1.3");
   f.println("id,AtomS3_Logger");
   f.println("description,M5AtomS3 + MPU6886 1kHz");
   f.println("orientation,YxZ");
   f.println("sample_rate,1000");
   f.println("tscale,0.000001");
   f.println("gscale,0.01745329252");
   f.println("ascale,1");
   f.println("t,gx,gy,gz,ax,ay,az");
 }
 void startLog() {
  timerAlarmDisable(timer);

  /* ファイル名決定 */
  uint32_t idx = 1;
  char fname[16];
  while (true) {
    sprintf(fname, "/%05lu.csv", (unsigned long)idx);
    if (!existsFile(fname)) break;
    idx++;
  }

  logFile = openFileWrite(fname);
  
  if (!logFile) {
    Serial.println("[SD] open failed");
    timerAlarmEnable(timer);
    return;
  }
  writeGcsvHeader(logFile);

  currentFileName = fname;
  logging_active  = true;
  last_sync       = millis();   // ★ 追加：初回同期まで 10 s 待たせる
  timerAlarmEnable(timer);
  Serial.printf("[LOG] start %s\n", fname);
}
 
void stopLog() {
  timerAlarmDisable(timer);

  if (logFile) {
     logFile.flush(); 
     sd.card()->syncDevice();
     logFile.close(); 
  }
  sd_dirty = false;
  logging_active = false;
  Serial.println("[LOG] stop");
  timerAlarmEnable(timer);   
  verifyLogFile();
}
 
 /* -------------- IMU キャリブレーション (3 s) ------------------------ */
 void calibrateImu()
 {
   Serial.println("[CAL] keep device still… 3 s");
   M5.Lcd.fillRect(0, 60, 128, 20, TFT_BLACK);
   M5.Lcd.setCursor(0, 60); M5.Lcd.print("Hold still 3s");
   delay(3000);
 
   Serial.println("[CAL] calibrating…");
   M5.Lcd.fillRect(0, 60, 128, 20, TFT_BLACK);
   M5.Lcd.setCursor(0, 60); M5.Lcd.print("Calibrating…");
 
   M5.Imu.setCalibration(100, 100, 0);
   M5.Imu.saveOffsetToNVS();
 
   Serial.println("[CAL] done");
   M5.Lcd.fillRect(0, 60, 128, 20, TFT_BLACK);
   M5.Lcd.setCursor(0, 60); M5.Lcd.print("Calib OK");
 }
 
 /* -------------------------------------------------------------------- */
 /*                                 setup                                */
 /* -------------------------------------------------------------------- */
 void setup()
 {
   Serial.begin(115200); 
   delay(100);
   M5.begin();
   M5.Lcd.setBrightness(150);
   M5.Lcd.fillScreen(TFT_BLACK);
   M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
   M5.Lcd.println("Init…");
   //SPIClass& bus = *M5.Lcd.getBus()->getSPI();   // ← LovyanGFX/M5Unified のバスを取得

   /* BtnA */
   M5.BtnA.setHoldThresh(3000);
 
   /* IMU 初期化 -------------------------------------------------------- */
   Wire.setClock(1000000);
   if (!M5.Imu.begin()) {
     M5.Lcd.println("[IMU] NG");
     while (1) yield();
   }
   Serial.println("[IMU] OK");
   M5.Lcd.println("[IMU] OK");
 
   /* SD 初期化 --------------------------------------------------------- */
   pinMode(SD_CS, OUTPUT);
   digitalWrite(SD_CS, HIGH);
 
   SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
 
   SdSpiConfig sdCfg(SD_CS, SHARED_SPI, SD_SCK_MHZ(12), &SPI);
   if (!sd.begin(sdCfg)) {
     M5.Lcd.println("[SD] NG");
     while (1) yield();
   }
   Serial.println("[SD] OK");
   M5.Lcd.println("[SD] OK");
 
   /* 容量表示 ---------------------------------------------------------- */
   uint64_t cardSizeMiB = (uint64_t)sd.card()->sectorCount() * 512ULL / (1024ULL * 1024ULL);
   Serial.printf("[SD] Card size: %llu MiB\n", cardSizeMiB);
 
   M5.Lcd.println("Press BtnA to start");
 
   /* タイマ初期化 ------------------------------------------------------ */
   timer = timerBegin(0, 80, true);
   timerAttachInterrupt(timer, &onTimer, true);
   timerAlarmWrite(timer, SAMPLE_US, true);
   timerAlarmEnable(timer);
 }
 
 /* -------------------------------------------------------------------- */
 /*                                 loop                                 */
 /* -------------------------------------------------------------------- */
 void loop()
 {
   M5.update();
 
   /* BtnA click → Start / Stop 切替 ---------------------------------- */
   if (M5.BtnA.wasClicked()) {
     Serial.println("[DEBUG] Button clicked!");
     logging_active ? stopLog() : startLog();
     Serial.printf("[DEBUG] After action, logging_active = %d\n", logging_active);
   }
 
   /* 長押し 3 s → キャリブ ------------------------------------------- */
   if (M5.BtnA.wasHold()) {
     calibrateImu();
   }
 
   /* 1 kHz サンプリング & ログ書込み -------------------------------- */
   if (sample_flag) {
     portENTER_CRITICAL(&timerMux);
     sample_flag = false;
     portEXIT_CRITICAL(&timerMux);
 
     float ax, ay, az, gx, gy, gz;
     M5.Imu.getAccel(&ax, &ay, &az);
     M5.Imu.getGyro(&gx, &gy, &gz);
 
     if (logging_active && logFile) {
       uint32_t t_us = micros();
       logFile.printf("%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                      t_us, gx, gy, gz, ax, ay, az);
       sd_dirty = true;
     }
 
     disp_ax = ax; disp_ay = ay; disp_az = az;
     disp_mag = sqrtf(ax * ax + ay * ay + az * az);
   }
 
   /* LCD 10 Hz 更新 --------------------------------------------------- */
   uint32_t now_ms = millis();
   if (now_ms - last_lcd_ms >= 100) {
     last_lcd_ms = now_ms;
     M5.Lcd.setCursor(0, 80);
     M5.Lcd.printf("Ax:%5.2f Ay:%5.2f\n", disp_ax, disp_ay);
     M5.Lcd.printf("Az:%5.2f |a|:%5.2f ", disp_az, disp_mag);
   }
    
    if (sd_dirty && (now_ms - last_sync >= SYNC_INTERVAL_MS)) {
      logFile.flush();              // FAT バッファへ
      sd.card()->syncDevice();      // 物理セクタへ
      last_sync  = millis();
      sd_dirty   = false;           // ★ 同期完了
    }
   }
 