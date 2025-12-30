#include <Wire.h>
#include <SPI.h>
#include <ArduCAM.h>
#include "memorysaver.h"

// --- 핀 설정 ---
#define CS_PIN 5
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define SDA_PIN 21
#define SCL_PIN 22

ArduCAM myCAM(OV2640, CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.println("\n=== ArduCAM Ultra-Stable Diagnostic Mode ===");

  // 1. 하드웨어 초기화
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  // I2C 속도 하향 (안정성 최우선)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(50000); 

  // SPI 속도를 1MHz로 대폭 하향 (배선 품질 무관하게 작동 유도)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS_PIN);
  SPI.setFrequency(1000000); 

  // 2. SPI 통신 테스트 (반복 확인)
  bool spi_ok = false;
  for(int i=0; i<5; i++) {
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    if (myCAM.read_reg(ARDUCHIP_TEST1) == 0x55) {
      spi_ok = true;
      break;
    }
    delay(200);
  }

  if (!spi_ok) {
    Serial.println("1. SPI [FAILED]");
    Serial.println("   -> 전원 부족 또는 접촉 불량입니다.");
    while (1);
  }
  Serial.println("1. SPI [OK]");

  // 3. 센서 인식
  uint8_t vid, pid;
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
  
  if ((vid != 0x26) || (pid != 0x42 && pid != 0x41)) {
    Serial.print("2. I2C [FAILED] ID: "); Serial.print(vid, HEX); Serial.print(":"); Serial.println(pid, HEX);
    while (1);
  }
  Serial.println("2. I2C [OK]");

  // 4. 카메라 초기화 및 안정화 지연
  myCAM.InitCAM();
  myCAM.set_format(JPEG);
  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
  delay(2000); // 초기화 후 전원 안정화 대기
  myCAM.clear_fifo_flag();
  
  Serial.println("3. System Ready. Starting Diagnostic Stream...\n");
}

int frameCount = 0;
int failCount = 0;

void loop() {
  // 캡처 전 전원 안정화 및 버퍼 정리
  delay(500); 
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  
  // 캡처 명령
  myCAM.start_capture();

  unsigned long start = millis();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    if (millis() - start > 2000) {
      Serial.println("[Timeout] Capture not finishing.");
      return;
    }
    yield();
  }

  uint32_t len = myCAM.read_fifo_length();
  
  // 153608 같은 고정된 큰 숫자는 데이터가 아니라 '쓰레기 값'일 확률이 높음
  if (len > 0 && len < 0x7FFFFF) {
    frameCount++;
    Serial.print("[Frame "); Serial.print(frameCount);
    Serial.print("] Size: "); Serial.print(len); 

    // 헤더 확인
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();
    uint8_t h1 = SPI.transfer(0x00);
    uint8_t h2 = SPI.transfer(0x00);
    myCAM.CS_HIGH();

    Serial.print(" | Header: "); Serial.print(h1, HEX); Serial.print(" "); Serial.print(h2, HEX);

    if (h1 == 0xFF && h2 == 0xD8) {
      Serial.println(" ✅ OK!");
      failCount = 0;
    } else {
      Serial.println(" ❌ BAD");
      failCount++;
      
      // 5번 연속 실패 시 카메라 재초기화 시도
      if (failCount >= 5) {
        Serial.println("   >>> 5회 연속 실패: 카메라 재설정 중...");
        myCAM.InitCAM();
        myCAM.set_format(JPEG);
        myCAM.OV2640_set_JPEG_size(OV2640_320x240);
        delay(1000);
        failCount = 0;
      }
      
      Serial.println("   [원인 진단]");
      if (len == 153608) {
        Serial.println("   -> Size가 153608로 고정됨: 전압 부족으로 인해 카메라 칩이 멈췄습니다.");
        Serial.println("   -> 해결: ESP32의 5V(VCC) 핀에 카메라를 연결하세요.");
      } else {
        Serial.println("   -> 데이터 오염: SPI 배선을 더 짧게 하거나 전원을 보강하세요.");
      }
    }
  } else {
    Serial.print("[Error] Invalid Size: "); Serial.println(len);
  }
}
