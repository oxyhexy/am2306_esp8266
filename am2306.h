#ifndef am2306_h
#define am2306_h

//Signalling:
//
//            Initiation of interaction      Sensor response             Binary useful data from the sensor        End of interaction
//                    (host)                    (sensor)                              (sensor)                          (sensor)
//          //======================//   //=================//   //=============================================//   //========//
//         //                      //   //                 //   //             0                    1          //   //        //
//        //                      //   //                 //   //     //--------------//   //--------------// //   //        //
//START: S1[L(800-20000)+H(20-200)] + S2[L(75-85)+H(75-85)] + S3[40*[[L(48-55)+H(22-30)]||[L(48-55)+H(68-75)]]] + S4[L(45-55)] :END
//           /    \                     /        /                \                     /                        /
//        MIN      MAX                 /        /          NUMBER OF TIMES            OR                        /
//          DURATION         STATE (L-LOW, H-HIGH)            Each time                                        /
//       (microseconds)                                      we get 1 bit                                     STAGE
//

//Timings (microseconds):
//

//Stage 1 (S1) "Initiation" (host):
//
#define MIN_S1_L 800
#define MAX_S1_L 20000
#define MIN_S1_H 20  //
#define MAX_S1_H 200 //Bus release.
//
//

//Stage 2 (S2) "Response start" (sensor):
//
#define MIN_S2_L 75
#define MAX_S2_L 85
#define MIN_S2_H 75
#define MAX_S2_H 85
//
//

//Stage 3 (S3) "Useful data" (sensor):
//
//First part of...
#define MIN_S3_L 48
#define MAX_S3_L 69 //Default is 55 microseconds, but really can be higher.
//Finally 0:
#define MIN_S3_H_0 22
#define MAX_S3_H_0 30
//Finally 1:
#define MIN_S3_H_1 68
#define MAX_S3_H_1 75
//
//There will be 16 bits of humidity, then 16 bits of temperature, then 8 bits of parity - 40 bits at all.
//The bits are represented as follows:
//Bit "0" - the sensor sets a low level and holds it for ~48-55 microseconds, then returns to a high level and holds it for ~22-30 microseconds.
//Bit "1" - the sensor sets a low level and holds it for ~48-55 microseconds, then returns to a high level and holds it for ~68-75 microseconds.
//
//

//Stage 4 (S4) "End of signal" (sensor):
//
#define MIN_S4_L 45 //
#define MAX_S4_L 55 //Bus release.
//
//

byte interrupts_counter;
unsigned long signal_level_change_timestamps[256]; //We actually need a capacity for timestamps vault equals interrupts counter value for the full signal, but in reality we can't guarantee that there won't be more for some reason. It is better not to burden any additional work for check any conditions to the function that is performed when an interrupt is triggered. Therefore, we need to have an array capacity for the timestamps of at least 256 values. In this case, no matter how many interrupts occur, there will be no overflow, because the interrupts counter has a byte type, which means that it will simply zero out when the value of 255 is exceeded. It's a minimal cyclic buffer somehow we can have that never go out of itself automatically and doesn't have to be cleaned - only interrupts counter -> 0 is enough.

//When the signal level changes, we will record a timestamp for that event:
void ICACHE_RAM_ATTR ISR_event() { signal_level_change_timestamps[interrupts_counter++] = micros(); }

//It's a function:
byte get_relative_humidity_and_celsius_temperature_from_am2306_sensor(const byte io_pin, int* humidity_container, int* temperature_container) {

  interrupts_counter = 0; //Reset.

  pinMode(io_pin, OUTPUT); //Set the pin to output mode.
  digitalWrite(io_pin, HIGH); //Set high level.

  attachInterrupt(digitalPinToInterrupt(io_pin), ISR_event, CHANGE); //Here we activate the signal "recording".

  //Here we "send" the starting signal to the sensor:
  digitalWrite(io_pin, LOW); //Set low level.
  delay(1); //Waiting for 1 millisecond.
  digitalWrite(io_pin, HIGH); //Set high level again.

  //Let's "listen" to the sensor response for 5 milliseconds, because it should fit in ~4:
  pinMode(io_pin, INPUT); //Set the pin to input mode.
  delay(5); //Waiting.
  detachInterrupt(digitalPinToInterrupt(io_pin)); //Stop "recording" the signal.

  if (interrupts_counter != 86) { return 1; } //The first criterion of correctness is the number of interruptions. If it is not equal to 86, then something is wrong.

  byte i = 0;

  unsigned int signal_levels_durations[85];

  //Convert the timestamps of the signal level changes into durations:
  for (i = 0; i < interrupts_counter-1; i++) {

    if (signal_level_change_timestamps[i+1] < signal_level_change_timestamps[i]) { signal_levels_durations[i] = ULONG_MAX - signal_levels_durations[i] + signal_levels_durations[i+1]; continue; } //The second criterion of correctness is the condition that the next timestamp must always be greater than the previous. If it is not, then the timer overflow has occurred and it has passed through zero. In this case we will get incorrect result. But we can fix it.
    signal_levels_durations[i] = signal_level_change_timestamps[i+1]-signal_level_change_timestamps[i]; //Signal level duration is the difference between the two nearest timestamps of the signal level change.

  }

  //Next is a protocol correctness checking and data extracting:
  byte compliance_counter = 0;

  //Checking host initial (in fact, here we do unnecessary work and check our generated signal, but let it be so... for the composition...):
  if (signal_levels_durations[0] >= MIN_S1_L  && signal_levels_durations[0] <= MAX_S1_L)  { compliance_counter++; }
  if (signal_levels_durations[1] >= MIN_S1_H  && signal_levels_durations[1] <= MAX_S1_H)  { compliance_counter++; }

  //Sensor response begins:
  if (signal_levels_durations[2] >= MIN_S2_L && signal_levels_durations[2] <= MAX_S2_L) { compliance_counter++; }
  if (signal_levels_durations[3] >= MIN_S2_H && signal_levels_durations[3] <= MAX_S2_H) { compliance_counter++; }

  if (compliance_counter < 4) { return 2; } //Wrong situation. Error in the initial part of the signal.

  //Sensor's initial response signal looks ok. Time for data:
  byte n=7; //Bits pointer/counter.

  //High 8 bits of humidity:
  byte high_byte_of_humidity;

  for (i = 4; i < 19; i += 2) {

    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_0 && signal_levels_durations[i+1] <= MAX_S3_H_0) { compliance_counter++; high_byte_of_humidity &= ~(1 << n--); } //0
    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_1 && signal_levels_durations[i+1] <= MAX_S3_H_1) { compliance_counter++; high_byte_of_humidity |= (1 << n--); } //1

  }

  n=7;

  //Low 8 bits of humidity:
  byte low_byte_of_humidity;

  for (i = 20; i < 35; i += 2) {

    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_0 && signal_levels_durations[i+1] <= MAX_S3_H_0 ) { compliance_counter++; low_byte_of_humidity &= ~(1 << n--); } //0
    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_1 && signal_levels_durations[i+1] <= MAX_S3_H_1 ) { compliance_counter++; low_byte_of_humidity |= (1 << n--); } //1

  }

  n=7;

  //High 8 bits of temperature:
  byte high_byte_of_temperature;

  for (i = 36; i < 51; i += 2) {

    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_0 && signal_levels_durations[i+1] <= MAX_S3_H_0 ) { compliance_counter++; high_byte_of_temperature &= ~(1 << n--); } //0
    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_1 && signal_levels_durations[i+1] <= MAX_S3_H_1 ) { compliance_counter++; high_byte_of_temperature |= (1 << n--); } //1

  }

  n=7;

  //Low 8 bits of temperature:
  byte low_byte_of_temperature;

  for (i = 52; i < 67; i += 2) {

    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_0 && signal_levels_durations[i+1] <= MAX_S3_H_0 ) { compliance_counter++; low_byte_of_temperature &= ~(1 << n--); } //0
    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_1 && signal_levels_durations[i+1] <= MAX_S3_H_1 ) { compliance_counter++; low_byte_of_temperature |= (1 << n--); } //1

  }

  n=7;

  //8 bits of parity:
  byte byte_of_parity;

  for (i = 68; i < 83; i += 2) {

    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_0 && signal_levels_durations[i+1] <= MAX_S3_H_0 ) { compliance_counter++; byte_of_parity &= ~(1 << n--); } //0
    if (signal_levels_durations[i] >= MIN_S3_L && signal_levels_durations[i] <= MAX_S3_L && signal_levels_durations[i+1] >= MIN_S3_H_1 && signal_levels_durations[i+1] <= MAX_S3_H_1 ) { compliance_counter++; byte_of_parity |= (1 << n--); } //1

  }

  if (compliance_counter < 44) { return 3; } //Wrong situation. Data extraction error.

  if (signal_levels_durations[84] >= MIN_S4_L && signal_levels_durations[84] <= MAX_S4_L) { compliance_counter++; } //The end...

  if (compliance_counter < 45) { return 4; } //Wrong situation. Problem with the end of the signal.

  //The protocol is correct. Let's check parity:
  if (byte_of_parity != (byte)(high_byte_of_humidity+low_byte_of_humidity+high_byte_of_temperature+low_byte_of_temperature)) { return 5; } //Parity check error. The data from the sensor may contain errors.

  //Combining bytes of humidity:
  int united_bytes_of_humidity = (high_byte_of_humidity << 8) | low_byte_of_humidity;
  //Combining bytes of temperature:
  int united_bytes_of_temperature = (high_byte_of_temperature << 8) | low_byte_of_temperature;

  //If the high bit is set to 1, the temperature value must be interpreted as negative.
  if (united_bytes_of_temperature & (1 << 15)) { united_bytes_of_temperature &= ~(1 << 15); united_bytes_of_temperature *= -1; } //So we set that bit to 0 and turn value into a minus.

  //Check for a valid range of values:
  if (united_bytes_of_humidity < 0 || united_bytes_of_humidity > 999 || united_bytes_of_temperature < -400  || united_bytes_of_temperature > 1250) { return 6; }; //Humidity: 0.0 to 99.9%RH, temperature: -40.0 to 125.0C(op.: -40 to 80C!).

  //Finally. Returning the result:
  *humidity_container = united_bytes_of_humidity;
  *temperature_container = united_bytes_of_temperature;
  return 0;

}

//Successful return:
//0 - the transferred to function variables now contain the values of humidity and temperature as an integer (to obtain a normal value with floating point, just divide by 10, if required).
//
//Errors:
//1 - interrupts counter error;
//2 - initial part of the signal error;
//3 - data bits extracting section error;
//4 - end part of the signal error;
//5 - parity error;
//6 - valid range error.

#endif
