//Libraries
#include <LiquidCrystal.h>

//define the order enumeration
enum Order {
  HELLO = 0,
  SERVO = 1,
  MOTOR = 2,
  ALREADY_CONNECTED = 3,
  ERROR = 4,
  RECEIVED = 5,
  STOP = 6,
};

int8_t num=78;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setup() {
  Serial.begin(115200);
  lcd.begin(20, 4);
  lcd.print("Debug Readout");
}

void loop() {
  if(Serial.available() > 0){
      int read_byte = Serial.read();
      lcd.setCursor(0, 1);
      lcd.print(read_byte);
      Order order_received = read_order(read_byte);
      lcd.setCursor(0, 2);
      lcd.print(order_received);

  }
}

Order read_order(int read_byte){
  return (Order) read_byte;
}
