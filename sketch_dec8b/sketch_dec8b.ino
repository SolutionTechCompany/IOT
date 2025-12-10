#define BLYNK_TEMPLATE_ID   "TMPL6SkrL5rkt"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN    "4jeqvNINdvc-1xOyfezkInm131IJil8T"

#define BLYNK_PRINT Serial0

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#define RX_PIN 18   // ESP32 RX  <- STM32 TX (PA9)
#define TX_PIN 17   // ESP32 TX  -> STM32 RX (PA10)

char ssid[] = "SolutionTech_2.4G";
char pass[] = "solutiontech031224";

BlynkTimer timer;
String uartBuf;

/* ============================================================
   BLYNK BUTTON V0 → ควบคุมวาล์วผ่าน STM32
   ============================================================ */
BLYNK_WRITE(V0) {
  int val = param.asInt();

  if (val == 1) {
    Serial0.println("-----------------------------Blynk: OPEN valve------------------------------------");
    Serial1.println("CMD_VALVE_ON");     // ส่งคำสั่งไป STM32
  } else {
    Serial0.println("-----------------------------Blynk: CLOSE valve-----------------------------------");
    Serial1.println("CMD_VALVE_OFF");
  }
}

/* ============================================================
   อ่านข้อมูลจาก STM32 แล้วส่งไป Blynk (V4, V5, V6)
   ============================================================ */
void readFromSTM32() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      String msg = uartBuf;
      uartBuf = "";

      msg.trim();
      Serial0.print("STM32 Raw: ");
      Serial0.println(msg);

      /* ---- Water state (V4) ---- */
      if (msg == "state: 1") {
        Blynk.virtualWrite(V4, 0);
        Serial0.println("→ Water NOT reached (V4 = 0)");
      }
      else if (msg == "state: 0") {
        Blynk.virtualWrite(V4, 1);
        Serial0.println("→ Water reached (V4 = 1)");
      }

      /* ---- Valve status (V6) ---- */
      else if (msg == "Relay = LOW (OFF)") {
        Blynk.virtualWrite(V6, 0);
        Serial0.println("→ Valve CLOSED");
      }
      else if (msg == "Relay = HIGH (ON)") {
        Blynk.virtualWrite(V6, 1);
        Serial0.println("→ Valve OPEN");
      }

      /* ---- Pressure from "ADC=...  P=... bar" (V5) ---- */
      else if (msg.startsWith("ADC=")) {
        // Find "P="
        int pIndex = msg.indexOf("P=");
        if (pIndex != -1) {
          // Take substring after "P="
          String pPart = msg.substring(pIndex + 2);    // skip "P="
          
          // Cut off the word "bar" if present
          int barIndex = pPart.indexOf("bar");
          if (barIndex != -1) {
            pPart = pPart.substring(0, barIndex);
          }

          pPart.trim();       // remove spaces
          float pBar = pPart.toFloat();   // e.g. "1.234" -> 1.234

          // Send to Blynk V5 as float (display widget set to float)
          Blynk.virtualWrite(V5, pBar);

          Serial0.print("→ Pressure = ");
          Serial0.println(pBar, 3);
        } else {
          Serial0.println("No P= found in ADC line");
        }
      }

      else {
        Serial0.println("Unknown msg, ignored");
      }
    }
    else if (c != '\r') {
      uartBuf += c;
    }
  }
}

void setup() {
  Serial0.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial0.println("\n=== ESP32 + STM32 UART + Blynk ===");

  Serial0.println("Sending CMD_VALVE_ON to STM32...");
  Serial1.println("CMD_VALVE_ON");

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  timer.setInterval(50L, readFromSTM32);
}

void loop() {
  Blynk.run();
  timer.run();
}
