/*LoRa - Node Adam */
/*
Hardware ID: stm32wle5xx
Model ID: rak3172
RUI API Version: 3.2.9
*/

#include <Arduino.h>
#include <Rak3172_Canopus.h>
#include "Canopus_Modbus.h"
ModbusMaster node;

uint8_t result;
long startTime;
bool rx_done = false;
double myFreq = 922500000;
uint16_t sf = 12, bw = 0, cr = 0, preamble = 8, txPower = 22;
unsigned long lastMillis = 0;
// Global flags and counters
bool awaiting_response = false;
uint8_t count_transmission = 0;
bool WakeupCallback = false;
//bool for sleep 
unsigned long previousMillis = 0;  // Lưu trữ thời gian gửi dữ liệu lần cuối
const unsigned long interval = 120000;  // Khoảng thời gian giữa các lần gửi (120000 ms = 2 phút)

//____________________ADAM___________________-
uint8_t result_analog;
uint8_t result_digital_in;

uint16_t rawValue;
float current;
uint16_t inputStatus;

// Khai báo biến global
uint8_t frame[16];
uint8_t len;
uint8_t temp_data[2];
char buff[16];

// Biến trạng thái
bool join_sent = false;      // Biến trạng thái để theo dõi việc gửi function 0x01
bool join_accepted = false;  // Biến trạng thái để theo dõi việc nhận phản hồi từ gateway
bool check_response = false;
// Địa chỉ MAC của node
uint8_t MAC[6] = { 0x0A, 0x0B, 0x0C, 0x0D, 0x11, 0x22 };  // Thay bằng địa chỉ MAC thực tế của bạn

//________________________________OneWire - DS18B20 __________________________
// Định nghĩa các chân và các hàm hỗ trợ
#define ONEWIRE_PIN_DATA GPIO_PIN_11
#define ONEWIRE_PORT_DATA GPIOA

void set_pin_as_output(GPIO_TypeDef *port, uint16_t pin) {
  // Cấu hình chân GPIO thành chế độ xuất (output).
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void set_pin_as_input(GPIO_TypeDef *port, uint16_t pin) {
  // Cấu hình chân GPIO thành chế độ nhập (input).
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

// Đặt lại DS18B20
void onewire_reset(void) {
  // Đặt lại cảm biến DS18B20.
  set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);                  // Cấu hình chân dữ liệu thành chế độ xuất.
  HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);  // Kéo chân dữ liệu xuống mức thấp.
  delayMicroseconds(480);                                                  // Chờ 480 microseconds.
  HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);    // Kéo chân dữ liệu lên mức cao.
  delayMicroseconds(480);                                                  // Chờ thêm 480 microseconds.
  set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);                   // Cấu hình chân dữ liệu thành chế độ nhập.
  delayMicroseconds(480);                                                  // Chờ thêm 480 microseconds.
}

// Ghi 1 bit vào DS18B20
void onewire_write_bit(uint8_t bit) {
  set_pin_as_output(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  if (bit) {  //Nếu bit la 1
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
    delayMicroseconds(1);
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
    delayMicroseconds(60);
  } else {
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
    delayMicroseconds(60);
    HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
    delayMicroseconds(1);
  }
  set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
}

// Đọc 1 bit từ DS18B20
uint8_t onewire_read_bit(void) {
  uint8_t bit = 0;
  set_pin_as_output(ONEWIRE_PORT_7DATA, ONEWIRE_PIN_DATA);
  HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_RESET);
  delayMicroseconds(1);
  HAL_GPIO_WritePin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA, GPIO_PIN_SET);
  set_pin_as_input(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  delayMicroseconds(15);
  bit = HAL_GPIO_ReadPin(ONEWIRE_PORT_DATA, ONEWIRE_PIN_DATA);
  delayMicroseconds(45);
  return bit;
}

// Ghi 1 byte vào DS18B20
void onewire_write_byte(uint8_t data) {
  // Ghi một byte vào DS18B20.
  for (uint8_t i = 0; i < 8; i++) {
    onewire_write_bit(data & 0x01);  // Ghi bit thấp nhất của byte.
    data >>= 1;                      // Dịch chuyển byte sang phải 1 bit.
  }
}

// Đọc 1 byte từ DS18B20
uint8_t onewire_read_byte(void) {
  // Đọc một byte từ DS18B20.
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    data |= (onewire_read_bit() << i);  // Đọc từng bit và dịch chuyển vào byte.
  }
  return data;  // Trả về giá trị byte đọc được.
}


// Đọc nhiệt độ từ DS18B20
float read_temperature(void) {
  uint8_t temp_LSB, temp_MSB;
  int16_t temp;

  // Khởi động lại cảm biến
  onewire_reset();

  // Gửi lệnh Skip ROM
  onewire_write_byte(0xCC);

  // Gửi lệnh Convert T
  onewire_write_byte(0x44);

  // Chờ cảm biến hoàn thành quá trình chuyển đổi nhiệt độ
  delay(750);

  // Khởi động lại cảm biến
  onewire_reset();

  // Gửi lệnh Skip ROM
  onewire_write_byte(0xCC);

  // Gửi lệnh Read Scratchpad
  onewire_write_byte(0xBE);

  // Đọc 2 byte dữ liệu nhiệt độ
  temp_LSB = onewire_read_byte();
  temp_MSB = onewire_read_byte();

  // Chuyển đổi dữ liệu nhiệt độ
  temp = ((int16_t)temp_MSB << 8) | temp_LSB;

  // Trả về giá trị nhiệt độ dạng float
  return (float)temp / 16.0;
}

//_________________________________END SETUP ONEWIRE___________________________

/*
Ham tinh gia tri CRC
Hàm tính giá trị CRC giúp kiểm tra tính toàn vẹn của dữ liệu.
*/
uint8_t crc8(const uint8_t *data, int len) {
  unsigned crc = 0;
  int i, j;
  for (j = 0; j < len; j++, data++) {
    crc ^= (*data << 8);
    for (i = 8; i > 0; --i) {
      if (crc & 0x8000)
        crc ^= (0x1070 << 3);
      crc <<= 1;
    }
  }
  return (uint8_t)(crc >> 8);
}

// Hàm hexDump để hiển thị nội dung buffer
void hexDump(uint8_t *buf, uint16_t len) {
  for (uint16_t i = 0; i < len; i += 16) {  //Duyet qua buffer theo tung nhom 16bytes
    char s[len];                            //Mang s: Luu tru ky tu ASCII
    uint8_t iy = 0;
    for (uint8_t j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        if (c > 31 && c < 128)
          s[iy++] = c;
      }
    }
    String msg = String(s);
    Serial.println(msg);
  }
  Serial.println("Buffer!");
}

// Callback khi nhận dữ liệu
void recv_cb(rui_lora_p2p_recv_t data) {
  rx_done = true;  // Chuyển Rx true để sẵn sàng nhận tín hiệu response từ việc callback
  if (data.BufferSize == 0) {
    Serial.println("Empty buffer.");
    return;
  }
  sprintf(buff, "Incoming message, length: %d, RSSI: %d, SNR: %d",
          data.BufferSize, data.Rssi, data.Snr);
  Serial.println(buff);
  hexDump(data.Buffer, data.BufferSize);
  digitalWrite(LED_RECV, !digitalRead(LED_RECV));
}

// Callback khi gửi dữ liệu thành công
void send_cb(void) {
  Serial.printf("P2P set Rx mode %s\r\n",
                api.lora.precv(65534) ? "Success" : "Fail");
}

// Gửi frame và quản lý trạng thái LED
void send_frame(uint8_t frame_type, uint8_t function, uint8_t *data, uint8_t data_len) {
  len = 0;                              //must have for reset len each call function
  frame[len++] = 0xFE;                  // BEGIN
  frame[len++] = 0;                     // LEN (placeholder)
  frame[len++] = frame_type;            // FRAME TYPE
  frame[len++] = function;              // FUNCTION
  memcpy(&frame[len], MAC, 6);          // MAC
  len += 6;                             //Mac address 6 bytes : cộng dồn
  memcpy(&frame[len], data, data_len);  // DATA
  len += data_len;

  // Tính độ dài thực tế của frame không bao gồm BEGIN, END, CRC và LEN
  frame[1] = len - 2;  // Gán giá trị thực tế cho LEN
  Serial.println();
  Serial.print("Len of frame: ");
  Serial.println(frame[1], HEX);

  uint8_t crc = crc8(&frame[1], len - 1);  // Tính CRC từ LEN đến hết data
  frame[len++] = crc;                      // CRC
  frame[len++] = 0xEF;                     // END

  // Truyền frame qua LoRa
  bool send_result = false;
  uint8_t retry_count = 0;        // Đếm số lần thử gửi lại.
  const uint8_t max_retries = 5;  // Giới hạn số lần thử gửi lại.

  while (!send_result && retry_count < max_retries) {
    send_result = api.lora.psend(len, frame);
    Serial.printf("P2P send %s\r\n", send_result ? "Success" : "Fail");
    if (!send_result) {
      Serial.printf("P2P finish Rx mode %s\r\n", api.lora.precv(0) ? "Success" : "Fail");
    }
    retry_count++;  // Tăng số lần thử gửi lại.
    Serial.print("Check resend: ");
    Serial.println(retry_count);
  }

  if (send_result) {
    Serial.printf("P2P send Success\r\n");
    digitalWrite(LED_SEND, !digitalRead(LED_SEND));
  }

  // Truyền frame
  Serial.print("Sending frame (Serial): ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}


//________________________________Sleep mode --- RAK3172___________________________
void sleep_mode() {
  Serial.print("The timestamp before sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
  Serial.println("(Wait 60 seconds or Press any key to wakeup)");
  api.system.sleep.all(60000);
  Serial.print("The timestamp after sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
}

//________________________________ADAM_________________________________
void read_ADAM() {
  //________________________Config ADAM____________________-
  // Đọc dữ liệu từ thanh ghi 40000 đến 40005 (analog inputs)
  result_analog = node.readHoldingRegisters(0, 1);  // Đọc từ 40000 đến 40005
  delay(10);
  if (result_analog == node.ku8MBSuccess) {  // Nếu đọc thành công
    rawValue = node.getResponseBuffer(0);
    current = (rawValue * 20.0) / 4095.0;  //value of current!
    Serial.printf("\r\nValue 4000%d: %d -> Current: %.2f mA", rawValue, current);
  } else {
    Serial.printf("\nRead Fail node 1");  // Nếu đọc thất bại
  }

  result_digital_in = node.readDiscreteInputs(0x00, 2);  // Read input status from address 0x0000
  delay(10);
  if (result_digital_in == node.ku8MBSuccess) {
    inputStatus = node.getResponseBuffer(0);
    Serial.printf("\nReceived Data: 0x%04X", inputStatus);
    for (uint8_t i = 0; i < 2; i++) {
      bool state = (inputStatus >> i) & 0x01;
      Serial.printf("\nDI_%02d: %s", i + 1, state ? "ON" : "OFF");  // active low
    }
  } else {
    Serial.printf("\nRead Fail, Error: %02X", result_digital_in);  // If reading fails
  }
  //___________________End read ADAM_________________________
}

void setup() {
  Serial.begin(115200);
  Serial.println("_______RAK3172_Canopus lora P2P");
  Serial.println("------------------------------------------------------");
  Serial.println("_______RAKwireless System Powersave");
  Serial.println("------------------------------------------------------");
  delay(2000);
  init_io();  //Enable I/O
  enable_Vss5();
  Serial_Canopus.begin(9600, SERIAL_8N1);
  enable_Vrs485();
  startTime = millis();
  //_______________________ONE WIRE____________________--
  if (api.lora.nwm.get() != 0) {
    Serial.printf("Set Node device work mode %s\r\n",
                  api.lora.nwm.set() ? "Success" : "Fail");
    api.system.reboot();
  }
  Serial.println("P2P Start");
  Serial.printf("Hardware ID: %s\r\n", api.system.chipId.get().c_str());
  Serial.printf("Model ID: %s\r\n", api.system.modelId.get().c_str());
  Serial.printf("RUI API Version: %s\r\n",
                api.system.apiVersion.get().c_str());
  Serial.printf("Firmware Version: %s\r\n",
                api.system.firmwareVersion.get().c_str());
  Serial.printf("AT Command Version: %s\r\n",
                api.system.cliVersion.get().c_str());
  Serial.printf("Set P2P mode frequency %3.3f: %s\r\n", (myFreq / 1e6),
                api.lora.pfreq.set(myFreq) ? "Success" : "Fail");
  Serial.printf("Set P2P mode spreading factor %d: %s\r\n", sf,
                api.lora.psf.set(sf) ? "Success" : "Fail");
  Serial.printf("Set P2P mode bandwidth %d: %s\r\n", bw,
                api.lora.pbw.set(bw) ? "Success" : "Fail");
  Serial.printf("Set P2P mode code rate 4/%d: %s\r\n", (cr + 5),
                api.lora.pcr.set(cr) ? "Success" : "Fail");
  Serial.printf("Set P2P mode preamble length %d: %s\r\n", preamble,
                api.lora.ppl.set(preamble) ? "Success" : "Fail");
  Serial.printf("Set P2P mode tx power %d: %s\r\n", txPower,
                api.lora.ptp.set(txPower) ? "Success" : "Fail");
  api.lora.registerPRecvCallback(recv_cb);
  api.lora.registerPSendCallback(send_cb);
  Serial.printf("P2P set Rx mode %s\r\n",
                api.lora.precv(65534) ? "Success" : "Fail");
  // Node -> Gateway - REQUEST
  if (!join_sent) {
    // Gửi function 0x01 (join request) chỉ một lần
    uint8_t join_data[1] = { 0x02};       // Node dùng cảm biến rớt + onewire
    send_frame(0x01, 0x01, join_data, 1);  // FRAME TYPE 0x01 (request), FUNCTION 0x01 (join)
    join_sent = true;                      // Đánh dấu là đã gửi
    Serial.println("_____________________________________");
    Serial.println("__________Join to Gateway____________");
  }
  delay(10);
}


/*
_____________________________CAU HINH FRAME TRUYEN_______________________________________
 [BEGIN]    [LEN]   [FRAME TYPE]    [FUNCTION]    [MAC ADDR]    [DATA]    [CRC]   [END]

BEGIN: OXFE
END  : OXEF
MAC  : MAC ADDRESS NODE
LEN  : Length frame
CRC  : CRC 8 bit frame
*/
void loop() {
  // Thiết lập ID slave cho node Modbus
  node.begin(1, Serial_Canopus);  // ID slave là 1
  unsigned long currentMillis = millis();
  // Gateway -> Node
  if (join_sent && !join_accepted && rx_done && !check_response) {
    // Nhận phản hồi từ gateway - Đã gửi tín hiệu request - nhưng chưa join được
    check_response = true;
    rx_done = false;
    uint8_t response = buff[5];  // Giả sử phản hồi từ gateway nằm ở byte thứ 6

    if (response == 0x00 || response == 0x01) {  // Kiểm tra phản hồi từ gateway
      join_accepted = true;                      // Đánh dấu là đã nhận phản hồi thành công
    } else {
      Serial.println("Join request failed. Gateway don't response ");
    }
  }
  //Nếu tín hiệu join vào gateway được chấp nhận
  if (join_accepted )
  {
    float temperature = read_temperature();
    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.println(" *C");

    int16_t temp_scaled = (temperature * 10);
    uint8_t temp_data[2];
    temp_data[0] = temp_scaled & 0xFF;
    temp_data[1] = (temp_scaled >> 8) & 0xFF;
    send_frame(0x03, 0x04, temp_data, 2);
    delay(10);

    read_ADAM();
    int16_t current_scaled = current * 10;
    // Gửi function 0x05 (ADAM - giả sử đây là sự kiện Digital Input/Output)
    uint8_t adam_data[4];
    adam_data[0] = (inputStatus >> 0) & 0x01;  // DO_1
    adam_data[1] = (inputStatus >> 1) & 0x01;  // DO_2
    adam_data[2] = current_scaled & 0xFF;    // Dữ liệu analog (thay đổi theo yêu cầu)
    adam_data[3] = (current_scaled >> 8) & 0xFF;        // LSB của dòng điện
    send_frame(0x03, 0x05, adam_data, 4);      // FRAME TYPE 0x03 (event data), 

    
    Serial.print("Adam Data 0: ");
    Serial.println(adam_data[0]);
  
    Serial.print("Adam Data 1: ");
    Serial.println(adam_data[1]);

    Serial.print("Adam Data 2: ");
    Serial.println(adam_data[2]);
  
    Serial.print("Adam Data 3: ");
    Serial.println(adam_data[3]);

    delay(30000);

  }
}