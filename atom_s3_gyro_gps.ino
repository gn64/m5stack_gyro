/*************************************************************************
 * AtomS3_Gyroflow_Logger.ino  – SD＋IMU calib＋speed
 *************************************************************************/
 #include <SPI.h>
 #include <SdFat.h>
 #include <Wire.h>
 #include "M5AtomS3.h"
 #include <math.h>
 
 /* ---------- SD ---------- */
 constexpr int SD_SCK  = 7;
 constexpr int SD_MOSI = 6;
 constexpr int SD_MISO = 8;
 constexpr int SD_CS   = 1;
 SdFs      sd;
 FsFile    logFile;
 bool      sd_dirty   = false;
 uint32_t  last_sync  = 0;
 constexpr uint32_t SYNC_INTERVAL_MS = 1000;   // flush 間隔 1 s
 
 /* ---------- IMU / Timer ---------- */
 constexpr uint32_t SAMPLE_US = 1000;          // 1 kHz
 volatile bool sample_flag    = false;
 hw_timer_t*  timer           = nullptr;
 portMUX_TYPE timerMux        = portMUX_INITIALIZER_UNLOCKED;
 
 float g_off[3] = {0.0f, 0.0f, 0.0f};   // ジャイロ (dps)
 float a_off[3] = {0.0f, 0.0f, 0.0f};   // 加速度 (g)
 
 /* ---------- Logging ---------- */
 String currentFileName = "";
 bool   logging_active  = false;
 
 /* ---------- LCD monitor ---------- */
 float  disp_ax=0, disp_ay=0, disp_az=0, disp_mag=0;
 float  speed_xy_kmh = 0.0f;
 uint32_t last_lcd_ms = 0;
 
 /* ---------- RAM log buffer ---------- */
 static char  buf[256*48];
 size_t wp = 0;
 
 /* ---------- Velocity integrator ---------- */
 float vx = 0.0f, vy = 0.0f;
 constexpr float G2MPS2 = 9.80665f;
 constexpr float DT = SAMPLE_US / 1e6f;        // 0.001 s
 void integrateSpeed(float ax_g, float ay_g) {
   vx += ax_g * G2MPS2 * DT;
   vy += ay_g * G2MPS2 * DT;
   speed_xy_kmh = sqrtf(vx*vx + vy*vy) * 3.6f;
 }
 
 
 /* ------------------------------------------------------------------ */
 /* SdFat helper                                                       */
 /* ------------------------------------------------------------------ */
 bool existsFile(const char* path){ return sd.exists(path); }
 
 FsFile openFileWrite(const char* path){
   FsFile f = sd.open(path, O_WRITE | O_CREAT | O_APPEND);
   if(!f){
     Serial.printf("[SD] open failed: %s\n", path);
     sd.printSdError(&Serial);
   }
   return f;
 }
 
 /* ------------------- verify --------------------------------------- */
 void verifyLogFile(){
   if(currentFileName.isEmpty()) return;
   if(!existsFile(currentFileName.c_str())){
       Serial.printf("[VERIFY] %s NOT FOUND\n", currentFileName.c_str()); return;}
   FsFile f = sd.open(currentFileName.c_str(), O_READ);
   Serial.printf("[VERIFY] %s OK (%u bytes)\n",
                 currentFileName.c_str(), (unsigned)f.size());
   f.close();
 }
 
 /* ------------------- ISR 1 kHz ------------------------------------ */
 void IRAM_ATTR onTimer(){
   portENTER_CRITICAL_ISR(&timerMux);
   sample_flag = true;
   portEXIT_CRITICAL_ISR(&timerMux);
 }
 
 /* ------------------- CSV header ----------------------------------- */
 void writeGcsvHeader(FsFile &f){
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
 
 /* ------------------- log buffer flush ----------------------------- */
 void flushBuf(){
   if(wp && logFile){
     if(logFile.write(buf, wp) != wp){
        Serial.print("[SD] write err "); sd.printSdError(&Serial);
     }
     wp = 0;
     sd_dirty = true;
   }
 }
 
 /* ------------------- start / stop --------------------------------- */
 void startLog(){
   timerAlarmDisable(timer);
 
   /* file name */
   uint32_t idx=1; char fname[16];
   while(true){ sprintf(fname,"/%05lu.csv",(unsigned long)idx);
                if(!existsFile(fname)) break; idx++;}
 
   logFile = openFileWrite(fname);
   if(!logFile){ timerAlarmEnable(timer); return; }
   writeGcsvHeader(logFile);
 
   currentFileName = fname;
   logging_active  = true;
   last_sync       = millis();
   vx = vy = speed_xy_kmh = 0.0f;   // zero velocity every log
   timerAlarmEnable(timer);
 
   Serial.printf("[LOG] start %s\n", fname);
   M5.Lcd.fillRect(0,0,128,12,TFT_BLACK);
   M5.Lcd.setCursor(0,0); M5.Lcd.print("▶ "); M5.Lcd.print(fname);
 }
 
 void stopLog(){
   timerAlarmDisable(timer);
   flushBuf();
   if(logFile){ logFile.flush(); sd.card()->syncDevice(); logFile.close();}
   sd_dirty = false; logging_active=false;
   Serial.println("[LOG] stop");
   verifyLogFile();
   timerAlarmEnable(timer);
 
   M5.Lcd.fillRect(0,0,128,12,TFT_BLACK);
   M5.Lcd.setCursor(0,0); M5.Lcd.print("■ stopped");
 }
 
 /* ------------------- calib save / load ---------------------------- */
 const char* CALIB_PATH = "/calib.txt";
 
 void saveCalibration(float gx, float gy, float gz,
                      float ax, float ay, float az){
   FsFile f = openFileWrite(CALIB_PATH);
   if(f){
      f.printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
               gx,gy,gz,ax,ay,az);
      f.close();
      Serial.println("[CAL] saved to /calib.txt");
   }
 }
 
 bool loadCalibration(){
   if(!existsFile(CALIB_PATH)) return false;
   FsFile f = sd.open(CALIB_PATH, O_READ);
   char line[96] = {0};
   int n = f.readBytesUntil('\n', line, sizeof(line)-1);
   f.close();
   if (n <= 0) return false;
   return 6 == sscanf(line, "%f,%f,%f,%f,%f,%f",
           &g_off[0], &g_off[1], &g_off[2],
           &a_off[0], &a_off[1], &a_off[2]);
 }
 
 /* ------------------- calib procedure ------------------------------ */
 void calibrateImu(){
   M5.Lcd.setCursor(0,60); M5.Lcd.print("Calibrating…");
   const int N = 500;                        // 0.5 秒
   float gx=0,gy=0,gz=0,ax=0,ay=0,az=0;
   for (int i=0;i<N;i++){
      float tgx,tgy,tgz,tax,tay,taz;
      M5.Imu.getGyro(&tgx,&tgy,&tgz);
      M5.Imu.getAccel(&tax,&tay,&taz);
      gx+=tgx; gy+=tgy; gz+=tgz;
      ax+=tax; ay+=tay; az+=taz-1.0f; // 1 g を引いて静止補正
      delay(1);
   }
   g_off[0]=gx/N; g_off[1]=gy/N; g_off[2]=gz/N;
   a_off[0]=ax/N; a_off[1]=ay/N; a_off[2]=az/N;
   vx = 0.0f;
   vy = 0.0f;
   speed_xy_kmh = 0.0f;          // LCD と積算速度をゼロクリア
 
   /* 保存 */
   FsFile f = openFileWrite(CALIB_PATH);
   if (f) {
       f.printf("%f,%f,%f,%f,%f,%f\n",
                g_off[0],g_off[1],g_off[2],
                a_off[0],a_off[1],a_off[2]);
       f.close();
   }
   M5.Lcd.fillRect(0,60,128,20,TFT_BLACK);
   M5.Lcd.setCursor(0,60); M5.Lcd.print("Calib OK");
   
 }
 
 /* ------------------- setup ---------------------------------------- */
 void setup(){
   Serial.begin(115200);
   delay(100);
   M5.begin();
   M5.Lcd.setBrightness(150);
   M5.Lcd.fillScreen(TFT_BLACK);
   M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
   M5.Lcd.println("Init…");
 
   /* BtnA */
   M5.BtnA.setHoldThresh(3000);
 
   /* IMU */
   Wire.setClock(1000000);
   if(!M5.Imu.begin()){ M5.Lcd.println("[IMU] NG"); while(1) yield(); }
   Serial.println("[IMU] OK");  M5.Lcd.println("[IMU] OK");
 
   /* SD */
   pinMode(SD_CS,OUTPUT); digitalWrite(SD_CS,HIGH);
   SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
   SdSpiConfig sdCfg(SD_CS, SHARED_SPI, SD_SCK_MHZ(12), &SPI);
   if(!sd.begin(sdCfg)){ M5.Lcd.println("[SD] NG"); while(1) yield(); }
   Serial.println("[SD] OK");   M5.Lcd.println("[SD] OK");
 
   /* calib auto-load */
   if(loadCalibration()) M5.Lcd.println("Calib: SD");
   else                  M5.Lcd.println("Calib: default");
 
   M5.Lcd.println("BtnA click = start");
 
   /* timer 1 kHz */
   timer = timerBegin(0,80,true);
   timerAttachInterrupt(timer,&onTimer,true);
   timerAlarmWrite(timer, SAMPLE_US, true);
   timerAlarmEnable(timer);
 }
 
 /* ------------------- loop ----------------------------------------- */
 void loop(){
   M5.update();
 
   /* button */
   if(M5.BtnA.wasClicked()){ logging_active? stopLog(): startLog(); }
   if(M5.BtnA.wasHold()){ calibrateImu(); }
 
   /* 1 kHz sample */
   if(sample_flag){
      portENTER_CRITICAL(&timerMux); sample_flag=false; portEXIT_CRITICAL(&timerMux);
 
      float ax,ay,az,gx,gy,gz;
      M5.Imu.getAccel(&ax,&ay,&az);
      M5.Imu.getGyro (&gx,&gy,&gz);
     /* ---------- ★★ ② 補正をここで計算 ---------- */
     float gx_c = gx - g_off[0];
     float gy_c = gy - g_off[1];
     float gz_c = gz - g_off[2];
     float ax_c = ax - a_off[0];
     float ay_c = ay - a_off[1];
     float az_c = az - a_off[2];
 
      /* logging buffer */
      if(logging_active && logFile){
         uint32_t t_us = micros();
         int n = snprintf(&buf[wp], sizeof(buf)-wp,
                        "%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        t_us, gx_c, gy_c, gz_c, ax_c, ay_c, az_c);
         if(n>0) wp+=n;
         if(wp > sizeof(buf)-80) flushBuf();
      }
 
      /* speed integration */
      integrateSpeed(ax_c, ay_c);
 
     disp_ax = ax_c;  disp_ay = ay_c;   // LCD 表示も補正後
     disp_az = az_c;  disp_mag = sqrtf(ax_c*ax_c + ay_c*ay_c + az_c*az_c);
   }
 
   /* LCD 10 Hz */
   uint32_t now_ms = millis();
   if(now_ms-last_lcd_ms >= 100){
      last_lcd_ms = now_ms;
      M5.Lcd.fillRect(0, 80, 128, 20, TFT_BLACK);
      M5.Lcd.setCursor(0,80);
      M5.Lcd.printf("Ax:%5.2f Ay:%5.2f\n", disp_ax, disp_ay);
      M5.Lcd.printf("Az:%5.2f |a|:%5.2f ", disp_az, disp_mag);
      M5.Lcd.printf("Vxy:%5.1f km/h", speed_xy_kmh);
   }
 
   /* periodic flush */
   if(sd_dirty && now_ms-last_sync>=SYNC_INTERVAL_MS){
      flushBuf();
      logFile.flush(); sd.card()->syncDevice();
      last_sync = now_ms; sd_dirty=false;
   }
 }
 