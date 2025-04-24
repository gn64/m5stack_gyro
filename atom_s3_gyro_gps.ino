/**
 * @file AtomS3_Gyroflow_Logger.ino
 * @brief Gyroflow互換のジャイロデータを記録するプログラム
 * 
 * このプログラムはAtomS3とAtomic GPS Baseを使用して、
 * Gyroflow互換のジャイロデータと時間同期情報を記録します。
 * バッテリー最適化機能も含まれています。
 * 
 * @hardware:
 * - M5AtomS3
 * - Atomic GPS Base
 * 
 * @libraries:
 * - M5AtomS3: https://github.com/m5stack/M5AtomS3
 * - TinyGPSPlus: https://github.com/mikalhart/TinyGPSPlus
 */

 #include "M5AtomS3.h"
 #include <TinyGPSPlus.h>
 #include <WiFi.h>
 #include <SPI.h>
 #include <SD.h>
 #include <time.h>
 #include <M5GFX.h>
 #include "esp_sleep.h"
 #include "esp_pm.h"
 #include "driver/rtc_io.h"
 
 using namespace m5gfx::ili9341_colors;  
 
 // デバイス設定
 #define GPS_RX_PIN 5      // AtomS3とAtomic GPS Baseの接続ピン（要確認）
 #define GPS_TX_PIN -1     // Atomic GPS BaseのRXピンは接続されていない
 #define GPS_BAUD_RATE 9600
 #define MPU6886_ADDR 0x68 // MPU6886 I2Cアドレス
 #define GYRO_LOG_INTERVAL 1000    // ジャイロセンサー取得間隔（us）- 1000Hz
 #define DISPLAY_UPDATE_INTERVAL 5000 // 画面更新間隔（ms）
 #define BATTERY_SAVE_DELAY 30000    // 省電力モードに入るまでの非アクティブ時間（ms）
 #define GYRO_BUFFER_SIZE 1000      // ジャイロデータバッファサイズ
 #define LOG_FILENAME "/GYRODATA.CSV" // Gyroflow互換ログファイル名
 #define KEEP_ALIVE_INTERVAL 25000    // TailBATの自動電源オフ防止間隔（ms）- 30秒以内
 #define KEEP_ALIVE_PIN 38            // 負荷をかけるためのピン
 
 // メニュー関連定数
 #define MENU_NONE 0
 #define MENU_MAIN 1
 #define MENU_SETTINGS 2
 
 // 定数
 #define GYRO_SCALE_FACTOR (1000.0f * 3.14159265358979323846f / 180.0f) // deg/s -> mrad/s
 
 // オブジェクト初期化
 TinyGPSPlus gps;
 HardwareSerial GPSSerial(1);  // HardwareSerial1を使用
 File logFile;
 bool sdCardAvailable = false;
 unsigned long lastGyroLogTime = 0;
 unsigned long lastDisplayUpdateTime = 0;
 unsigned long lastActivityTime = 0;
 unsigned long lastKeepAliveTime = 0;  // TailBAT自動電源オフ防止用タイマー
 bool timeSet = false;
 bool lowPowerMode = false;
 bool displayOn = true;
 bool recordingActive = false;
 unsigned long recordStartTime = 0;
 
 // センサー状態管理
 bool gyroEnabled = true;     // ジャイロセンサーの有効/無効状態
 bool gpsEnabled = true;      // GPSセンサーの有効/無効状態
 
 // メニュー管理
 uint8_t currentMenu = MENU_NONE;
 uint8_t menuSelection = 0;
 unsigned long lastMenuUpdateTime = 0;
 
 // 時間表示用バッファ
 char timeStr[32];
 char dateStr[32];
 
 // ジャイロデータ構造体
 struct GyroData {
   int64_t timestamp;  // マイクロ秒単位のタイムスタンプ
   float gyroX, gyroY, gyroZ;  // mrad/s単位
   float accX, accY, accZ;     // g単位
 };
 
 // ジャイロデータバッファ
 GyroData gyroBuffer[GYRO_BUFFER_SIZE];
 int bufferIndex = 0;
 bool bufferFull = false;
 
 // メニュー処理関数
 void handleMenu() {
   // ボタン操作の検出
   bool shortPress = M5.BtnA.wasPressed();
   bool longPress = M5.BtnA.pressedFor(1000);
   
   if (shortPress || longPress) {
     lastActivityTime = millis();  // アクティビティタイマーをリセット
     
     if (longPress) {
       // 長押し：選択項目を次へ移動
       menuSelection = (menuSelection + 1) % 4;
       
       // メニュー更新
       if (currentMenu == MENU_MAIN) {
         showMainMenu();
       } else if (currentMenu == MENU_SETTINGS) {
         showSettingsMenu();
       }
     } 
     else if (shortPress) {
       // 短押し：選択項目を実行
       switch (currentMenu) {
         case MENU_MAIN:
           handleMainMenuSelection();
           break;
         case MENU_SETTINGS:
           handleSettingsMenuSelection();
           break;
         case MENU_NONE:
           // 通常画面からメインメニューへ
           currentMenu = MENU_MAIN;
           menuSelection = 0;
           showMainMenu();
           break;
       }
     }
   }
 }
 
 // メインメニューの選択処理
 void handleMainMenuSelection() {
   switch (menuSelection) {
     case 0:  // Start/Stop Recording
       recordingActive = !recordingActive;
       if (recordingActive) {
         startRecording();
       } else {
         stopRecording();
       }
       currentMenu = MENU_NONE;  // メニューを閉じる
       updateDisplay();
       break;
     case 1:  // Settings
       currentMenu = MENU_SETTINGS;
       menuSelection = 0;
       showSettingsMenu();
       break;
     case 2:  // Status
       showStatusScreen();
       delay(3000);  // 3秒間表示
       if (currentMenu == MENU_MAIN) {
         showMainMenu();
       }
       break;
     case 3:  // Back to Main Screen
       currentMenu = MENU_NONE;
       updateDisplay();
       break;
   }
 }
 
 // 設定メニューの選択処理
 void handleSettingsMenuSelection() {
   switch (menuSelection) {
     case 0:  // Gyro Sensor On/Off
       gyroEnabled = !gyroEnabled;
       setGyroEnabled(gyroEnabled);
       showSettingsMenu();
       break;
     case 1:  // GPS Sensor On/Off
       gpsEnabled = !gpsEnabled;
       setGpsEnabled(gpsEnabled);
       showSettingsMenu();
       break;
     case 2:  // Reset Settings
       // 設定をデフォルトに戻す
       gyroEnabled = true;
       gpsEnabled = true;
       setGyroEnabled(true);
       setGpsEnabled(true);
       showSettingsMenu();
       break;
     case 3:  // Back to Main Menu
       currentMenu = MENU_MAIN;
       menuSelection = 0;
       showMainMenu();
       break;
   }
 }
 
 // 録画開始処理
 void startRecording() {
   recordStartTime = millis();
   
   // 新しいファイルを作成
   if (sdCardAvailable) {
     // 既存ファイルがあれば削除
     if (SD.exists(LOG_FILENAME)) {
       SD.remove(LOG_FILENAME);
     }
     writeGyroflowCsvHeader();
   }
   
   // バッファをリセット
   bufferIndex = 0;
   bufferFull = false;
   
   // 必要なセンサーが有効になっていることを確認
   if (!gyroEnabled) {
     setGyroEnabled(true);
   }
   
   Serial.println("Recording started");
 }
 
 // 録画停止処理
 void stopRecording() {
   // バッファの残りを書き込み
   writeGyroBufferToSD();
   
   Serial.println("Recording stopped");
 }// メインメニューを表示する関数
 void showMainMenu() {
   M5.Lcd.fillScreen(BLACK);
   M5.Lcd.setCursor(0, 0);
   M5.Lcd.setTextColor(YELLOW);
   M5.Lcd.println("== MAIN MENU ==");
   M5.Lcd.setTextColor(WHITE);
   
   // メニュー項目を表示
   const char* menuItems[] = {
     "Start/Stop Recording",
     "Settings",
     "Status",
     "Back to Main Screen"
   };
   
   for (int i = 0; i < 4; i++) {
     if (i == menuSelection) {
       M5.Lcd.setTextColor(GREEN);
       M5.Lcd.print("> ");
     } else {
       M5.Lcd.setTextColor(WHITE);
       M5.Lcd.print("  ");
     }
     M5.Lcd.println(menuItems[i]);
   }
   
   // 操作方法を表示
   M5.Lcd.setTextColor(BLUE);
   M5.Lcd.println("\nShort press: Select");
   M5.Lcd.println("Long press: Next");
 }
 
 // 設定メニューを表示する関数
 void showSettingsMenu() {
   M5.Lcd.fillScreen(BLACK);
   M5.Lcd.setCursor(0, 0);
   M5.Lcd.setTextColor(YELLOW);
   M5.Lcd.println("== SETTINGS ==");
   M5.Lcd.setTextColor(WHITE);
   
   // メニュー項目を表示
   const char* menuItems[] = {
     "Gyro Sensor: ",
     "GPS Sensor: ",
     "Reset Settings",
     "Back to Main Menu"
   };
   
   for (int i = 0; i < 4; i++) {
     if (i == menuSelection) {
       M5.Lcd.setTextColor(GREEN);
       M5.Lcd.print("> ");
     } else {
       M5.Lcd.setTextColor(WHITE);
       M5.Lcd.print("  ");
     }
     
     M5.Lcd.print(menuItems[i]);
     
     // 状態表示
     if (i == 0) {
       M5.Lcd.println(gyroEnabled ? "ON" : "OFF");
     } else if (i == 1) {
       M5.Lcd.println(gpsEnabled ? "ON" : "OFF");
     } else {
       M5.Lcd.println();
     }
   }
   
   // 操作方法を表示
   M5.Lcd.setTextColor(BLUE);
   M5.Lcd.println("\nShort press: Select");
   M5.Lcd.println("Long press: Next");
 }
 
 // ステータス画面を表示する関数
 void showStatusScreen() {
   M5.Lcd.fillScreen(BLACK);
   M5.Lcd.setCursor(0, 0);
   M5.Lcd.setTextColor(YELLOW);
   M5.Lcd.println("== STATUS ==");
   M5.Lcd.setTextColor(WHITE);
   
   // センサー状態
   M5.Lcd.print("Gyro: ");
   M5.Lcd.println(gyroEnabled ? "ON" : "OFF");
   
   M5.Lcd.print("GPS: ");
   M5.Lcd.println(gpsEnabled ? "ON" : "OFF");
   
   // GPS情報
   if (gpsEnabled) {
     M5.Lcd.print("Satellites: ");
     if (gps.satellites.isValid()) {
       M5.Lcd.println(gps.satellites.value());
     } else {
       M5.Lcd.println("--");
     }
     
     M5.Lcd.print("Fix: ");
     M5.Lcd.println(gps.location.isValid() ? "Yes" : "No");
   }
   
   // バッテリー情報（AtomS3では直接取得できないため概算値）
   M5.Lcd.print("Battery: ");
   M5.Lcd.println("N/A");
   
   // SD情報
   M5.Lcd.print("SD Card: ");
   M5.Lcd.println(sdCardAvailable ? "OK" : "None");
   
   // 戻る方法
   M5.Lcd.setTextColor(BLUE);
   M5.Lcd.println("\nPress to return");
 }// ジャイロセンサーの電源制御関数
 void setGyroEnabled(bool enable) {
   if (enable && !gyroEnabled) {
     // ジャイロセンサーをオンにする
     if (M5.Imu.begin()) {
       gyroEnabled = true;
       Serial.println("Gyro sensor enabled");
     } else {
       Serial.println("Failed to enable gyro sensor");
     }
   } 
   else if (!enable && gyroEnabled) {
     // ジャイロセンサーをオフにする
     // MPU6886をスリープモードに設定する
     //M5.In_I2C.beginTransmission(MPU6886_ADDR);
     //M5.In_I2C.write(0x6B);  // PWR_MGMT_1 レジスタ
     //M5.In_I2C.write(0x40);  // スリープモードに設定（0x40 = 01000000）
     //M5.In_I2C.endTransmission(true);
     M5.In_I2C.writeRegister8(MPU6886_ADDR, 0x6B, 0x40, 400000);  // 400kHz
 
     gyroEnabled = false;
     Serial.println("Gyro sensor disabled");
   }
 }
 
 // GPSセンサーの電源制御関数
 void setGpsEnabled(bool enable) {
   if (enable && !gpsEnabled) {
     // GPSセンサーをオンにする
     GPSSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
     gpsEnabled = true;
     Serial.println("GPS sensor enabled");
   } 
   else if (!enable && gpsEnabled) {
     // GPSセンサーをオフにする
     GPSSerial.end();
     gpsEnabled = false;
     Serial.println("GPS sensor disabled");
   }
 }// TailBATの自動電源オフを防止する関数
 void keepTailBatAlive() {
   // LEDを点灯させて電流を消費（約20mA）
   digitalWrite(KEEP_ALIVE_PIN, HIGH);
   
   // ディスプレイを一時的に明るくする（約30mA追加）
   uint8_t currentBrightness = M5.Lcd.getBrightness();
   M5.Lcd.setBrightness(100);
   
   // 短い演算処理でCPUに負荷をかける（約10-15mA追加）
   volatile unsigned long sum = 0;
   for (int i = 0; i < 500000; i++) {
     sum += i;
   }
   
   // 合計で約60-65mA消費した後、元の状態に戻す
   delay(200);  // 200ms間負荷をかけ続ける
   digitalWrite(KEEP_ALIVE_PIN, LOW);
   M5.Lcd.setBrightness(currentBrightness);
   
   Serial.println("TailBAT keep-alive pulse sent");
 }
 
 // GPS時刻をマイクロ秒タイムスタンプに変換する関数
 int64_t gpsTimeToTimestamp(TinyGPSPlus& gps) {
   struct tm timeinfo;
   timeinfo.tm_year = gps.date.year() - 1900;
   timeinfo.tm_mon = gps.date.month() - 1;
   timeinfo.tm_mday = gps.date.day();
   timeinfo.tm_hour = gps.time.hour();
   timeinfo.tm_min = gps.time.minute();
   timeinfo.tm_sec = gps.time.second();
   
   // UTC時間をエポック秒に変換
   time_t epochSeconds = mktime(&timeinfo);
   
   // マイクロ秒に変換して時間の小数部を追加
   return (int64_t)epochSeconds * 1000000LL + (int64_t)(gps.time.centisecond() * 10000);
 }
 
 // 現在のマイクロ秒タイムスタンプを取得する関数
 int64_t getCurrentMicros() {
   struct timeval tv;
   gettimeofday(&tv, NULL);
   return (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
 }
 
 // MPU6886からデータを読み取り、バッファに保存する関数
 void readAndBufferGyroData() {
   if (!gyroEnabled) return; // ジャイロが無効の場合は何もしない
   
   float ax, ay, az, gx, gy, gz;
   M5.Imu.getAccelData(&ax, &ay, &az);
   M5.Imu.getGyroData(&gx, &gy, &gz);
   
   // タイムスタンプ取得（マイクロ秒）
   int64_t timestamp = getCurrentMicros();
   
   // バッファに保存（GyroFlowフォーマット）
   // 注: GyroFlowはmrad/s単位、MPU6886はdeg/s単位で出力するため変換が必要
   gyroBuffer[bufferIndex].timestamp = timestamp;
   gyroBuffer[bufferIndex].gyroX = gx * GYRO_SCALE_FACTOR; // deg/s -> mrad/s
   gyroBuffer[bufferIndex].gyroY = gy * GYRO_SCALE_FACTOR;
   gyroBuffer[bufferIndex].gyroZ = gz * GYRO_SCALE_FACTOR;
   gyroBuffer[bufferIndex].accX = ax; // g単位
   gyroBuffer[bufferIndex].accY = ay;
   gyroBuffer[bufferIndex].accZ = az;
   
   bufferIndex = (bufferIndex + 1) % GYRO_BUFFER_SIZE;
   if (bufferIndex == 0) {
     bufferFull = true;
   }
 }
 
 // 省電力モードを設定する関数
 void setLowPowerMode(bool enable) {
   if (enable && !lowPowerMode) {
     lowPowerMode = true;
     
     // CPU周波数を下げる
     esp_pm_config_esp32s3_t pm_config;
     pm_config.max_freq_mhz = 80;  // 周波数を80MHzに制限
     pm_config.min_freq_mhz = 40;  // 最低40MHzまで下げることを許可
     pm_config.light_sleep_enable = true;  // 軽いスリープを有効化
     esp_pm_configure(&pm_config);
     
     // ディスプレイを消す
     if (displayOn) {
       M5.Lcd.sleep();
       displayOn = false;
     }
     
     // 注意: 省電力モードでもTailBAT自動電源オフ防止は継続する
     
     Serial.println("Entering low power mode");
   } 
   else if (!enable && lowPowerMode) {
     lowPowerMode = false;
     
     // CPU周波数を戻す
     esp_pm_config_esp32s3_t pm_config;
     pm_config.max_freq_mhz = 240;  // 通常の最大周波数
     pm_config.min_freq_mhz = 80;   // 最低周波数
     pm_config.light_sleep_enable = false;
     esp_pm_configure(&pm_config);
     
     // ディスプレイを復帰
     if (!displayOn) {
       M5.Lcd.wakeup();
       displayOn = true;
       updateDisplay();  // 画面を更新
     }
     
     Serial.println("Exiting low power mode");
   }
 }
 
 // GPSから時刻を設定する関数
 void syncTimeFromGPS() {
   if (gps.time.isValid() && gps.date.isValid()) {
     struct tm timeinfo;
     
     timeinfo.tm_year = gps.date.year() - 1900;  // tm_yearは1900年からの年数
     timeinfo.tm_mon = gps.date.month() - 1;     // tm_monは0-11
     timeinfo.tm_mday = gps.date.day();
     timeinfo.tm_hour = gps.time.hour();
     timeinfo.tm_min = gps.time.minute();
     timeinfo.tm_sec = gps.time.second();
     
     time_t t = mktime(&timeinfo);
     struct timeval tv = { .tv_sec = t };
     settimeofday(&tv, NULL);
     
     timeSet = true;
     M5.Lcd.fillRect(0, 0, 128, 30, BLACK);
     M5.Lcd.setCursor(0, 0);
     M5.Lcd.println("Time sync OK!");
     Serial.println("Time synchronized from GPS.");
   }
 }
 
 // 現在の日時を文字列で取得
 void getTimeString() {
   struct tm timeinfo;
   if (getLocalTime(&timeinfo)) {
     strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
     strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", &timeinfo);
   } else {
     strcpy(timeStr, "Time not set");
     strcpy(dateStr, "Date not set");
   }
 }
 
 // Gyroflow互換のCSVヘッダーを書き込む関数
 void writeGyroflowCsvHeader() {
   if (!sdCardAvailable) return;
   
   logFile = SD.open(LOG_FILENAME, FILE_WRITE);
   if (!logFile) {
     Serial.println("Error opening log file!");
     return;
   }
   
   // Gyroflow互換のCSVヘッダー
   logFile.println("timestamp,gyro_x,gyro_y,gyro_z,accl_x,accl_y,accl_z");
   logFile.close();
   
   Serial.print("Log file created: ");
   Serial.println(LOG_FILENAME);
 }
 
 // バッファリングしたジャイロデータをSDカードに書き込む関数
 void writeGyroBufferToSD() {
   if (!sdCardAvailable || (!bufferFull && bufferIndex == 0)) return;
   
   logFile = SD.open(LOG_FILENAME, FILE_APPEND);
   if (!logFile) {
     Serial.println("Error opening log file!");
     return;
   }
   
   // バッファ内のデータを書き込み
   int count = bufferFull ? GYRO_BUFFER_SIZE : bufferIndex;
   int startIdx = bufferFull ? bufferIndex : 0;
   
   for (int i = 0; i < count; i++) {
     int idx = (startIdx + i) % GYRO_BUFFER_SIZE;
     
     // Gyroflowフォーマットでデータを書き込み
     logFile.print(gyroBuffer[idx].timestamp);
     logFile.print(",");
     logFile.print(gyroBuffer[idx].gyroX, 6);
     logFile.print(",");
     logFile.print(gyroBuffer[idx].gyroY, 6);
     logFile.print(",");
     logFile.print(gyroBuffer[idx].gyroZ, 6);
     logFile.print(",");
     logFile.print(gyroBuffer[idx].accX, 6);
     logFile.print(",");
     logFile.print(gyroBuffer[idx].accY, 6);
     logFile.print(",");
     logFile.println(gyroBuffer[idx].accZ, 6);
   }
   
   logFile.close();
   
   // バッファをリセット
   bufferIndex = 0;
   bufferFull = false;
 }
 
 // 画面に情報を表示する関数
 void updateDisplay() {
   if (currentMenu != MENU_NONE) return;  // メニュー表示中は更新しない
   
   M5.Lcd.fillScreen(BLACK);
   M5.Lcd.setCursor(0, 0);
   
   // 日時表示
   getTimeString();
   M5.Lcd.println(dateStr);
   M5.Lcd.println(timeStr);
   
   // 録画状態表示
   if (recordingActive) {
     M5.Lcd.setTextColor(RED);
     M5.Lcd.println("Recording...");
     
     // 経過時間表示
     unsigned long elapsedSec = (millis() - recordStartTime) / 1000;
     M5.Lcd.printf("%02d:%02d\n", (elapsedSec / 60) % 60, elapsedSec % 60);
     M5.Lcd.setTextColor(WHITE);
   } else {
     M5.Lcd.println("Ready");
     M5.Lcd.println("Press BTN for menu");
   }
   
   // センサー状態表示
   M5.Lcd.print("Gyro: ");
   M5.Lcd.println(gyroEnabled ? "ON" : "OFF");
   
   M5.Lcd.print("GPS: ");
   M5.Lcd.println(gpsEnabled ? "ON" : "OFF");
   
   // GPS情報表示（GPSが有効な場合）
   if (gpsEnabled && gps.location.isValid()) {
     M5.Lcd.print("Sats: ");
     if (gps.satellites.isValid()) {
       M5.Lcd.println(gps.satellites.value());
     } else {
       M5.Lcd.println("--");
     }
   }
   
   // ジャイロデータ表示（ジャイロが有効な場合）
   if (gyroEnabled) {
     float ax, ay, az, gx, gy, gz, t;
     M5.Imu.getAccelData(&ax, &ay, &az);
     M5.Imu.getGyroData(&gx, &gy, &gz);
     
     M5.Lcd.print("Gyro: ");
     M5.Lcd.print(gz, 0);
     M5.Lcd.println(" deg/s");
   }
   
   // SDカード使用状況
   if (sdCardAvailable) {
     M5.Lcd.print("SD: ");
     M5.Lcd.print(SD.cardSize() / (1024 * 1024));
     M5.Lcd.println("MB");
   } else {
     M5.Lcd.println("SD: Not found");
   }
 }
 
 // 初期化関数
 void setup() {
   // 省電力設定
   esp_pm_config_esp32s3_t pm_config;
   pm_config.max_freq_mhz = 240;  // 通常時の最大周波数
   pm_config.min_freq_mhz = 80;   // 通常時の最小周波数
   pm_config.light_sleep_enable = false;
   esp_pm_configure(&pm_config);
   
   // AtomS3の初期化（省電力オプション付き）
   auto cfg = M5.config();
   cfg.clear_display = true;      // 起動時の余計な表示をクリア
   cfg.output_power = true;       // 電源出力を有効化
   cfg.internal_imu = true;       // 内蔵IMUを使用
   cfg.internal_rtc = true;       // 内蔵RTCを使用
   cfg.internal_spk = false;      // スピーカーは無効化（省電力のため）
   cfg.internal_mic = false;      // マイクは無効化（省電力のため）
   
   M5.begin(cfg);
   
   // 画面設定
   M5.Lcd.setRotation(0);
   M5.Lcd.fillScreen(BLACK);
   M5.Lcd.setTextSize(1);
   M5.Lcd.setCursor(0, 0);
   M5.Lcd.println("Gyroflow Logger");
   M5.Lcd.println("Initializing...");
   
   // 不要なWi-FiとBluetoothを無効化して電力節約
   WiFi.mode(WIFI_OFF);
   btStop();
   
   // シリアル通信開始
   Serial.begin(115200);
   Serial.println("AtomS3 Gyroflow Logger - Power Optimized");
 
   
   // GPS初期化
   GPSSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
   Serial.println("GPS Serial initialized");
   M5.Lcd.println("GPS initialized");
   
   // MPU6886初期化 - Gyroflow用に高サンプリングレート設定
   if (M5.Imu.begin()) {
     Serial.println("MPU6886 initialized");
     M5.Lcd.println("MPU6886 initialized");
     // 必要に応じてジャイロセンサーの帯域幅やサンプリングレートを設定
   } else {
     Serial.println("MPU6886 init failed");
     M5.Lcd.println("IMU Error!");
   }
   
   // SDカード初期化
   sdCardAvailable = SD.begin();
   if (sdCardAvailable) {
     Serial.println("SD Card initialized");
     writeGyroflowCsvHeader();
     M5.Lcd.println("SD: OK");
   } else {
     Serial.println("SD Card initialization failed");
     M5.Lcd.println("SD: Not found");
   }
   
   // TailBAT自動電源オフ防止用ピンの設定
   pinMode(KEEP_ALIVE_PIN, OUTPUT);
   digitalWrite(KEEP_ALIVE_PIN, LOW);
 
   // 初期アクティビティ時間を設定
   lastActivityTime = millis();
   lastKeepAliveTime = millis();
   
   M5.Lcd.println("Ready!");
   delay(1000);
   
   // 画面初期化
   updateDisplay();
 }
 
 // メインループ
 void loop() {
   M5.update();  // ボタン状態更新
   
   unsigned long currentMillis = millis();
   
   // メニュー表示中はメニュー処理を行う
   if (currentMenu != MENU_NONE) {
     handleMenu();
   }
   // 通常モードでボタンが押された場合はメニューを表示
   else if (M5.BtnA.wasPressed()) {
     currentMenu = MENU_MAIN;
     menuSelection = 0;
     showMainMenu();
     lastActivityTime = currentMillis;
   }
   
   // TailBATの自動電源オフを防止する（約25秒ごと）
   if (currentMillis - lastKeepAliveTime >= KEEP_ALIVE_INTERVAL) {
     lastKeepAliveTime = currentMillis;
     keepTailBatAlive();
   }
   
   // GPSデータの読み込み（GPSが有効な場合）
   if (gpsEnabled) {
     bool newGpsDataReceived = false;
     while (GPSSerial.available() > 0) {
       char c = GPSSerial.read();
       if (gps.encode(c)) {
         newGpsDataReceived = true;
       }
     }
     
     // 新しいGPSデータがあれば、アクティビティタイマーをリセット
     if (newGpsDataReceived) {
       lastActivityTime = currentMillis;
     }
     
     // 時刻同期（GPS信号が有効で、まだ時刻設定がされていない場合）
     if (!timeSet && gps.time.isValid() && gps.date.isValid()) {
       syncTimeFromGPS();
       lastActivityTime = currentMillis;
     }
   }
   
   // 録画中はジャイロデータを高頻度で取得（ジャイロが有効な場合）
   if (recordingActive && gyroEnabled) {
     // マイクロ秒レベルのタイミングで測定
     unsigned long currentMicros = micros();
     if (currentMicros - lastGyroLogTime >= GYRO_LOG_INTERVAL) {
       lastGyroLogTime = currentMicros;
       
       // ジャイロデータ読み取り
       readAndBufferGyroData();
       
       // バッファがいっぱいになったら書き込み
       if (bufferFull) {
         writeGyroBufferToSD();
       }
     }
   }
   
   // 画面の定期的な更新（メニュー表示中でない場合）
   if (currentMenu == MENU_NONE && !lowPowerMode && displayOn && 
       currentMillis - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
     lastDisplayUpdateTime = currentMillis;
     updateDisplay();
   }
   
   // 録画中でなく、一定時間アクティビティがなければ省電力モードに移行
   if (!recordingActive && !lowPowerMode && currentMillis - lastActivityTime >= BATTERY_SAVE_DELAY) {
     setLowPowerMode(true);
   }
   
   // 省電力モードの軽量化のためのスリープ
   if (lowPowerMode) {
     // 軽いスリープを実行
     delay(100);
   }
 } 