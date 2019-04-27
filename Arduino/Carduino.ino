
#include <LiquidCrystal.h>

//Parameters
const int SERIAL_BAUD = 115200;  // Baudrate
const int MOTOR_PIN = 3;
const int DIRECTION_PIN = 4;
const int SERVOMOTOR_PIN = 6;
const int INITIAL_THETA = 110; // Initial angle of the servomotor
  // Min and max values for motors
const int THETA_MIN = 60;
const int THETA_MAX = 150;
const int SPEED_MAX = 100;

  // If DEBUG is set to true, the arduino will send back all the received messages
const bool DEBUG = false;

bool is_connected = false; ///< True if the connection with the master is available
int8_t motor_speed = 0;
int16_t servo_angle = INITIAL_THETA;
int test_led = 13;
//Servo servomotor;**

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

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup()
{
  // Init Serial
  Serial.begin(SERIAL_BAUD);

  // Init Motor
  /*pinMode(MOTOR_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  
  // Stop the car
  
  stop();
  */
  pinMode(test_led,OUTPUT);
  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  // Print a message to the LCD.
  lcd.print("hello, world!");

  // Init Servo
  //servomotor.attach(SERVOMOTOR_PIN);**
  // Order between 0 and 180
  //servomotor.write(INITIAL_THETA);**

/*  while(!is_connected)
  {
    write_order(HELLO);
    wait_for_bytes(1, 1000);
    get_messages_from_serial();
  }
  */

}

void loop(){
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  
  
  
  //get_messages_from_serial();
  //update_motors_orders();
}

/*!
 * \brief Send updated motors orders to the two motors (servomotor + motor)
 */
void update_motors_orders(){
  //servomotor.write(constrain(servo_angle, THETA_MIN, THETA_MAX));//can we somehow use the servo library to controll our dude?**
  motor_speed = constrain(motor_speed, -SPEED_MAX, SPEED_MAX);
  // Send motor speed order
  //code below to be used temporarily commented to test communication versatility
  /*if (motor_speed > 0)
  {
    digitalWrite(DIRECTION_PIN, LOW);
  }
  else
  {
    digitalWrite(DIRECTION_PIN, HIGH);
  }
  analogWrite(MOTOR_PIN, convert_to_pwm(float(motor_speed)));
  analogWrite(test_led,convert_to_pwm(float(motor_speed)));

  */
  
}

/*!
 * Stop the car (set the speed to 0)
 */
void stop(){
  analogWrite(MOTOR_PIN, 0);
  digitalWrite(DIRECTION_PIN, LOW);
}

/*!
 * \brief Convert a speed order (in percentage of max speed)
 * into a pwm order (between 0 and 255)
 * \param motor_speed speed order in percentage of max speed
 * \return the speed order in pwm
 */
int convert_to_pwm(float motor_speed){
  // TODO: compensate the non-linear dependency speed = f(PWM_Value)
  return (int) round(abs(motor_speed)*(255./100.));
}

/*!
 * \brief Listen the serial and decode the message received
 */
void get_messages_from_serial(){
  
  if(Serial.available() > 0){
    // The first byte received is the instruction
    int read_byte = Serial.read();
    lcd.setCursor(0, 2);
    lcd.print(read_byte);
    Order order_received = read_order(read_byte);
    lcd.setCursor(0, 3);
    lcd.print(order_received);

    if(order_received == HELLO)
    {
      
      // If the cards haven't say hello, check the connection
      if(!is_connected)
      {
        is_connected = true;
        write_order(HELLO);
      }
      else
      {
        // If we are already connected do not send "hello" to avoid infinite loop
        write_order(ALREADY_CONNECTED);
      }
    }
    else if(order_received == ALREADY_CONNECTED)
    {
      is_connected = true;
    }
    else
    {
      switch(order_received)
      {
        case STOP:
        {
          lcd.setCursor(0, 1);
          lcd.print("STOP");
          motor_speed = 0;
          stop();
          if(DEBUG)
          {
            write_order(STOP);
          }
          break;
        }
        case SERVO:
        {
          lcd.setCursor(0, 1);
          lcd.print("SERVO");
          servo_angle = read_i16();
          if(DEBUG)
          {
            write_order(SERVO);
            write_i16(servo_angle);
          }
          break;
        }
        case MOTOR:
        {
          lcd.setCursor(0, 1);
          lcd.print("MOTOR");
          // between -100 and 100
          motor_speed = read_i8();
          if(DEBUG)
          {
            write_order(MOTOR);
            write_i8(motor_speed);
          }
          break;
        }
        // Unknown order
        default:
          write_order(ERROR);
          write_i16(404);
          return;
      }
    }
    write_order(RECEIVED); // Confirm the reception
  }
  
}

/*!
 * \brief Read one byte from the serial and cast it to an Order
 * \return the order received
 */
Order read_order(int read_byte){
  return (Order) read_byte;
}

/*!
 * \brief Wait until there are enough bytes in the buffer
 * \param num_bytes the number of bytes
 * \param timeout (ms) The timeout, time after which we release the lock
 * even if there are not enough bytes
 */
void wait_for_bytes(int num_bytes, unsigned long timeout){
  unsigned long startTime = millis();
  //Wait for incoming bytes or exit if timeout
  while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)){}
}

/*!
 * \brief Read signed bytes and put them in a buffer
 * \param buffer an array of bytes
 * \param n number of bytes to read
 */
void read_signed_bytes(int8_t* buffer, size_t n){
  size_t i = 0;
  int c;
  while (i < n)
  {
    c = Serial.read();
    if (c < 0) break;
    *buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
    i++;
  }
}

/*!
 * \brief Read one byte from a serial port and convert it to a 8 bits int
 * \return the decoded number
 */
int8_t read_i8(){
  wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
  return (int8_t) Serial.read();
}

/*!
 * \brief Read two bytes from a serial port and convert it to a 16 bits int
 * \return the decoded number
 */
int16_t read_i16(){
  int8_t buffer[2];
  wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
  read_signed_bytes(buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

/*!
 * \brief Read four bytes from a serial port and convert it to a 32 bits int
 * \return the decoded number
 */
int32_t read_i32(){
  int8_t buffer[4];
  wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
  read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

/*!
 * \brief Send one order (one byte)
 * \param order type of order
 */
void write_order(enum Order myOrder){
  uint8_t* Order = (uint8_t*) &myOrder;
  Serial.write(Order, sizeof(uint8_t));
}

/*!
 * \brief Write one byte int to serial port (between -127 and 127)
 * \param num an int of one byte
 */
void write_i8(int8_t num){
  Serial.write(num);
}

/*!
 * \brief Send a two bytes signed int via the serial
 * \param num the number to send (max: (2**16/2 -1) = 32767)
 */
void write_i16(int16_t num){
  int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  Serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}

/*!
 * \brief Send a four bytes signed int (long) via the serial
 * \param num the number to send (−2,147,483,647, +2,147,483,647)
 */
void write_i32(int32_t num){
  int8_t buffer[4] = {(int8_t) (num & 0xff), (int8_t) (num >> 8 & 0xff), (int8_t) (num >> 16 & 0xff), (int8_t) (num >> 24 & 0xff)};
  Serial.write((uint8_t*)&buffer, 4*sizeof(int8_t));
}


