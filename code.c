#include <Wire.h> // I2C 통신 라이브러리
#include "MAX30105.h" // MAX30105 심박센서 라이브러리
#include "heartRate.h" // 심박수 측정 관련 함수 라이브러리
#include <SoftwareSerial.h> // 소프트웨어 시리얼 통신 라이브러리
#include <LiquidCrystal_I2C.h> // I2C LCD 라이브러리

MAX30105 particleSensor; // 심박센서 객체 생성
LiquidCrystal_I2C lcd(0x27,16,2); // I2C LCD 객체 생성
SoftwareSerial BTSerial(2,3); // 소프트웨어 시리얼 객체 생성 (블루투스 통신에 사용)

const byte RATE_SIZE = 4; // 심박수 측정에 사용되는 배열 크기
byte rates[RATE_SIZE]; // 심박수 측정을 위한 배열
byte rateSpot = 0; // 배열 인덱스 변수
long lastBeat = 0; // 마지막 심박 측정 시간

float beatsPerMinute; // 분당 심박수
int beatAvg; // 심박수 평균값
int beatAvglast = 0; // 이전 심박수 평균값

int buzzerPin = 2; // 부저 핀
int redPin = 11; // 빨간색 LED 핀
int greenPin = 12; // 초록색 LED 핀
int bluePin = 13; // 파란색 LED 핀

void setup()
{
  Serial.begin(9600); // 시리얼 통신 초기화
  BTSerial.begin(9600); // 소프트웨어 시리얼 통신 초기화
  
  Serial.println("Initializing..."); // 초기화 메시지 출력
  display.setBrightness(0x0f); // 디스플레이 밝기 설정 (최대)
  display.showNumberDec(0, false); // 디스플레이에 0 표시
  
  // 심박센서 초기화
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. "); // 심박센서 연결 여부 확인
    while (1); // 연결 오류 시 무한루프
  }
  Serial.println("Place your index finger on the sensor with steady pressure."); // 손가락을 센서 위에 안정적으로 누워두라는 안내 메시지 출력

  particleSensor.setup(); // 심박센서 기본 설정
  particleSensor.setPulseAmplitudeRed(0x0A); // 빨간색 LED 강도 설정 (낮은 값은 센서가 동작 중임을 나타냄)
  particleSensor.setPulseAmplitudeGreen(0); // 초록색 LED 끄기
  pinMode(redPin, OUTPUT); // 빨간색 LED 출력 설정
  pinMode(greenPin, OUTPUT); // 초록색 LED 출력 설정
  pinMode(bluePin, OUTPUT); // 파란색 LED 출력 설정
  lcd.init(); // LCD 초기화
  lcd.backlight(); // LCD 백라이트 켜기
}

void loop()
{
  long irValue = particleSensor.getIR(); // IR 값을 읽어옴

  if (BTSerial.available()) {
    Serial.write(BTSerial.read()); // 블루투스에서 데이터가 수신되면 시리얼 모니터에 출력
  }

  if (Serial.available()) {
    BTSerial.write(Serial.read()); // 시리얼 모니터에서 데이터가 수신되면 블루투스로 전송
  }

  if (checkForBeat(irValue) == true)
  {
    // 심박이 감지되었을 때
    long delta = millis() - lastBeat; // 심박 감지 간격 계산
    lastBeat = millis(); // 현재 시간으로 갱신

    beatsPerMinute = 60 / (delta / 1000.0); // 분당 심박수 계산

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; // 심박수를 배열에 저장
      rateSpot %= RATE_SIZE; // 배열 인덱스를 순환시킴

      // 저장된 심박수의 평균값 계산
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;

      if (irValue < 50000){
        display.showNumberDec(0, false); // IR 값이 임계값보다 작으면 디스플레이에 0 표시
      } else {
        display.showNumberDec(beatAvg, false); // 디스플레이에 평균 심박수 표시
        lcd.clear(); // LCD 화면 지우기
        lcd.setCursor(0, 0); // LCD 커서 위치 설정
        lcd.println(beatAvglast); // 이전 심박수 평균값 출력
        BTserial.print("현재 심박수 : ");
        BTserial.print(beatAvg);
        BTserial.print("이전 심박수 : ");
        BTserial.print(beatAvglast);
        BTserial.print("(단위 : bpm");
        delay(1000);
        beatAvglast = beatAvg; // 현재 심박수 평균값을 이전 값으로 저장

        if (beatAvg < 60 || beatAvg > 100) {
          // 비정상적인 심박수 범위에 대한 처리 코드를 여기에 추가해야 함
        }
      }
    }
  }
}

void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red); // 빨간색 LED 밝기 설정
  analogWrite(greenPin, green); // 초록색 LED 밝기 설정
  analogWrite(bluePin, blue); // 파란색 LED 밝기 설정
}