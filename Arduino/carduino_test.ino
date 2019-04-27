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
bool is_connected = false;
bool ard_reads_py = false;
bool py_reads_ard = false;

void setup() {
  Serial.begin(115200);
  lcd.begin(20, 4);
  lcd.print("Arduino Debug Readout");
  handshake();
  //Handshake Sequence
  
}

void handshake(){
Order broadcast = HELLO;
  while (!is_connected){
    write_order(broadcast);
    wait_for_bytes(1,1000);
    if(Serial.available() > 0){
      int read_byte = Serial.read();
      lcd.setCursor(0, 2);
      lcd.print(read_byte);
      Order order_received = read_order(read_byte);
      lcd.setCursor(0, 3);
      lcd.print(order_received);

      if(order_received == HELLO){
      
        // If the cards haven't say hello, check the connection
        if(!is_connected){
          is_connected = true;
          write_order(HELLO);
        }
        else{
          // If we are already connected do not send "hello" to avoid infinite loop
          write_order(ALREADY_CONNECTED);
        }
      }
      else if(order_received == ALREADY_CONNECTED){
        is_connected = true;
      }
    }
  }

}
void loop() {
if(Serial.available() > 0){
    lcd.setCursor(0, 1);
    lcd.print(Serial.read());
    lcd.setCursor(0, 2);
    lcd.print((Order)Serial.peek());
    delay(100);
  }
}

void write_order(enum Order myOrder){
  uint8_t* Order = (uint8_t*) &myOrder;
  Serial.write(Order, sizeof(uint8_t));
}

Order read_order(int read_byte){
  return (Order) read_byte;
}

void wait_for_bytes(int num_bytes, unsigned long timeout){
  unsigned long startTime = millis();
  //Wait for incoming bytes or exit if timeout
  while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)){}
}


