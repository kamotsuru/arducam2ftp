/*
  Arducam2FTP
  Posted 14 Apr. 2024 by Kenichi Ogaki     
*/
/*----------------------------------------------------------------------------/
/ TJpgDec - Tiny JPEG Decompressor R0.03                       (C)ChaN, 2021
/-----------------------------------------------------------------------------/
/ The TJpgDec is a generic JPEG decompressor module for tiny embedded systems.
/ This is a free software that opened for education, research and commercial
/  developments under license policy of following terms.
/
/  Copyright (C) 2021, ChaN, all right reserved.
/
/ * The TJpgDec module is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/----------------------------------------------------------------------------*/
/*
   FTP passive client for IDE v1.0.1 and w5100/w5200
   Posted October 2012 by SurferTim
   Modified 6 June 2015 by SurferTim
   Modified 23 Sep. 2023 by Kenichi Ogaki
*/

#include <WiFi.h>
#include <ArduCAM.h>
#include <Wire.h>
#include <SPI.h>

#include <WiFiUdp.h>
#include <Syslog.h>

#include <rom/tjpgd.h>

// Bytes per pixel of image output
#define JD_FORMAT 0
#define N_BPP (3 - JD_FORMAT)

// Buffer to store an ArduCAM captured jpeg
const int BUF_SIZE = 26624;
char jpeg_buffer[BUF_SIZE];

// ArduCAM driver handle with CS pin defined as pin 5 for ESP32 dev kits
const int CS = 5;
ArduCAM myCAM(OV2640, CS);

uint8_t *frame_buffer; /* TJpgDec Output frame buffer */

// TJpgDec Session identifier for input/output functions (name, members and usage are as user defined)
typedef struct {
    char *jbuf;             /* Input stream */
    uint8_t *fbuf;          /* Output frame buffer */
    unsigned int wfbuf;     /* Width of the frame buffer [pix] */
    uint32_t lbyte;         /* Number bytes left in the input stream */
    int diff;               /* difference between current and previous frame buffers */
} IODEV;

// TJpgDec User defined input function
size_t in_func (    /* Returns number of bytes read (zero on error) */
    JDEC* jd,       /* Decompression object */
    uint8_t* buff,  /* Pointer to the read buffer (null to remove data) */
    size_t nbyte    /* Number of bytes to read/remove */
)
{
    IODEV *dev = (IODEV*)jd->device;   /* Session identifier (5th argument of jd_prepare function) */

    if (nbyte > dev->lbyte)
      nbyte = dev->lbyte;
    if (buff) /* Raad data from imput stream */
      memcpy(buff, dev->jbuf, nbyte);
    dev->lbyte -= nbyte;
    dev->jbuf += nbyte;

    return nbyte;
}

// TJpgDec User defined output function
unsigned int out_func (      /* Returns 1 to continue, 0 to abort */
    JDEC* jd,       /* Decompression object */
    void* bitmap,   /* Bitmap data to be output */
    JRECT* rect     /* Rectangular region of output image */
)
{
    IODEV *dev = (IODEV*)jd->device;   /* Session identifier (5th argument of jd_prepare function) */
    uint8_t *src, *dst;
    uint16_t y, bws, i;
    unsigned int bwd;

    /* Progress indicator */
    if (rect->left == 0) {
        if(Serial)
          Serial.printf("\r%lu%%. ", (rect->top << jd->scale) * 100UL / jd->height);
    }
    /* Copy the output image rectangle to the frame buffer */
    src = (uint8_t*)bitmap;                           /* Output bitmap */
    dst = dev->fbuf + N_BPP * (rect->top * dev->wfbuf + rect->left);  /* Left-top of rectangle in the frame buffer */
    bws = N_BPP * (rect->right - rect->left + 1);     /* Width of the rectangle [byte] */
    bwd = N_BPP * dev->wfbuf;                         /* Width of the frame buffer [byte] */
    for (y = rect->top; y <= rect->bottom; y++) {
      for (i = 0; i < bws/N_BPP; i++){ // calculate the difference between current and previous frame buffers
        dev->diff += sq((int)dst[i*3] - (int)src[i*3]);
        dev->diff += sq((int)dst[i*3 + 1] - (int)src[i*3 + 1]);
        dev->diff += sq((int)dst[i*3 + 2] - (int)src[i*3 + 2]);
      }
      memcpy(dst, src, bws);   /* Copy a line */
      src += bws; dst += bwd;  /* Next line */
    }

    return 1;    /* Continue to decompress */
}

// Initializes the I2C interface to the arducam
//  The code after Wire.begin() isn't strictly necessary, but it verifies that the I2C interface is
//  working correctly and that the expected OV2640 camera is connected.
void arducam_i2c_init() {
  // Initialize I2C bus
  Wire.begin();

  // Verify the I2C bus works properly, and the arducam vid and pid registers match their
  // expected values
  uint8_t vid, pid;
  while(1) {
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) {
      if(Serial)
        Serial.println(F("I2C error!"));
      delay(1000);
      continue;
    }
    else {
      if(Serial)
        Serial.println(F("I2C initialized."));
      break;
    } 
  }
}

// Initializes the SPI interface to the arducam
//  The code after SPI.begin() isn't strictly necessary, but it implements some workarounds to
//  common problems and verifies that the SPI interface is working correctly
void arducam_spi_init() {
  // set the CS pin as an output:
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  // initialize SPI:
  SPI.begin();

  // Reset the CPLD register (workaround for intermittent spi errors)
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  // Check if the ArduCAM SPI bus is OK
  while (1) {
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    uint8_t temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55) {
      if(Serial)
        Serial.println(F("SPI error!"));
      delay(1000);
      continue;
    } else {
      if(Serial)
        Serial.println(F("SPI initialized."));
      break;
    }
  }
}

// Initializes the arducam driver and hardware with desired settings. Should run once during setup()
void arducam_init() {
    arducam_spi_init();
    arducam_i2c_init();
    // set to JPEG format, this works around issues with the color data when sampling in RAW formats
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    // Specify the smallest possible resolution
    myCAM.OV2640_set_JPEG_size(OV2640_640x480);
    delay(100);
    if(Serial)
      Serial.println(F("Camera initialized."));
}

// Capture a photo on the arducam. This method only takes the photograph, call arducam_transfer() to
// read get the data 
void arducam_capture() {
  // Make sure the buffer is emptied before each capture
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  // Start capture
  myCAM.start_capture();
  // Wait for indication that it is done
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  delay(50);
  myCAM.clear_fifo_flag();
}

// Transfer the last captured jpeg from the arducam to a buffer. The buffer must be larger than the
// captured jpeg. Returns the length of the jpeg file in bytes
uint32_t arducam_transfer(char buf[], uint32_t buf_len) {
  uint32_t jpeg_length = myCAM.read_fifo_length() - 1; //exclude first byte 0x00
  // verify buffer can fit the jpeg
  if (jpeg_length > buf_len) {
    if(Serial) {
      Serial.println(F("Error: buffer not large enough to hold image"));
      Serial.println(F(jpeg_length));
    }
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  buf[0] = SPI.transfer(0x00); //skip first byte 0x00
  for (int index = 0; index < jpeg_length; index++) {
    buf[index] = SPI.transfer(0x00);
  }
  delayMicroseconds(15);
  myCAM.CS_HIGH();

  return jpeg_length;
}

//wifi configuration
char* ssid     = "Your SSID";
char* password = "Your WiFi password";
IPAddress wifiIp;
WiFiClient client;
WiFiClient dclient;

// change to your server
IPAddress server( 192, 168, 0, 11 );

WiFiUDP udpClient;
Syslog syslog(udpClient, server, 514, "esp32");

char outBuf[128];
char outCount;

// change fileName to your file (8.3 format!)
char fileName[13];
int seqNum = 1;

void setup()
{
  // Set up the serial connection for printing to terminal
  Serial.begin(115200);
  if(Serial){
    Serial.println("Serial Interface Initialized.");
  }

  arducam_init();

  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {
      delay(500);
      if(Serial)
        Serial.print(".");
  }
  wifiIp = WiFi.localIP();
  if(Serial) {
    Serial.println("");
    Serial.println(WiFi.localIP());
    
    Serial.printf("Total heap: %d\n", ESP.getHeapSize());
    Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  }
  Wire.begin();

  frame_buffer = (uint8_t*)malloc(N_BPP * 640 * 480 / 16); /* Create TJpgDec frame buffer for 1/16 output image */
}

void loop()
{
  if(Serial)  {
    Serial.println();
    Serial.print(F("taking a photo in 3... "));
    delay(500);
    Serial.print(F("2... "));
    delay(500);
    Serial.print(F("1..."));
    delay(500);
    Serial.println();
    Serial.println(F("*click*"));
  }

  // Take the photo
  arducam_capture();

  // Transfer the photo to jpeg buffer 
  uint32_t jpeg_size = arducam_transfer(jpeg_buffer, BUF_SIZE);

  JRESULT res;      /* Result code of TJpgDec API */
  JDEC jdec;        /* TJpgDec Decompression object */
  void *work;       /* Pointer to the TJpgDec work area */
  size_t sz_work = 3100; /* Size of TJpgDec work area */
  IODEV devid;      /* TJpgDec Session identifier */

  /* Initialize TJpgDec input stream */
  devid.jbuf = jpeg_buffer;
  devid.lbyte = jpeg_size;
  devid.diff = 0;

  /* Prepare to TJpgDec decompress */
  work = (void*)malloc(sz_work);
  res = jd_prepare(&jdec, in_func, work, sz_work, &devid);
  if (res == JDR_OK) {
    /* It is ready to dcompress and image info is available here */
    if(Serial)
      Serial.printf("Image size is %u x %u.\n%u bytes of work ares is used.\n", jdec.width, jdec.height, sz_work - jdec.sz_pool);
    /* Initialize output device */
    devid.fbuf = frame_buffer;
    devid.wfbuf = jdec.width / 4; /* 1/4 scaling */

    res = jd_decomp(&jdec, out_func, 2);   /* Start to decompress with 1/4 scaling */
    if (res == JDR_OK) {
      /* Decompression succeeded. You have the decompressed image in the frame buffer here. */
      if(Serial)
        Serial.printf("\rDecompression succeeded.\n");
    } else {
      if(Serial)
        Serial.printf("jd_decomp() failed (rc=%d)\n", res);
    }
  } else {
    if(Serial)
      Serial.printf("jd_prepare() failed (rc=%d)\n", res);
  }
  free(work);             /* Discard work area */

  sprintf(fileName, "test%04d.jpg", seqNum);

  if (devid.diff > 20000000) { // ftp the current jpg image if the difference between current and previous and images is more than this value
    doFTP(jpeg_buffer, jpeg_size);
    syslog.logf(LOG_INFO, "diff = %d", devid.diff);
    seqNum++;
  }

  if(seqNum > 999)
    seqNum = 1;
  delay(10000);
}

byte doFTP(char buf[], uint32_t buf_length)
{
  if (client.connect(server,21)) {
    if(Serial)
      Serial.println(F("Command connected"));
  } 
  else {
    if(Serial)
      Serial.println(F("Command connection failed"));
    return 0;
  }

  if(!eRcv()) return 0;
  client.println(F("USER pi"));

  if(!eRcv()) return 0;
  client.println(F("PASS raspberrypi"));

  if(!eRcv()) return 0;
  client.println(F("SYST"));

  if(!eRcv()) return 0;
  client.println(F("Type I"));

  if(!eRcv()) return 0;
  client.println(F("CWD /home/pi/testimg"));

  if(!eRcv()) return 0;
  client.println(F("PASV"));

  if(!eRcv()) return 0;
  char *tStr = strtok(outBuf,"(,");
  int array_pasv[6];
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL,"(,");
    array_pasv[i] = atoi(tStr);
    if(tStr == NULL)
    {
      if(Serial)
        Serial.println(F("Bad PASV Answer"));
    }
  }

  unsigned int hiPort,loPort;

  hiPort = array_pasv[4] << 8;
  loPort = array_pasv[5] & 255;

  if(Serial)
    Serial.print(F("Data port: "));
  hiPort = hiPort | loPort;
  if(Serial)
    Serial.println(hiPort);

  if (dclient.connect(server,hiPort)) {
    if(Serial)
      Serial.println(F("Data connected"));
  } 
  else {
    if(Serial)
      Serial.println(F("Data connection failed"));
    client.stop();
    return 0;
  }

  client.print(F("STOR "));
  client.println(fileName);

  if(!eRcv())
  {
    dclient.stop();
    return 0;
  }

  if(Serial){
    Serial.print(fileName);
    Serial.println(F(" Writing"));
  }
  byte clientBuf[64];
  int clientCount = 0, bufferCount = 0;

  while(bufferCount < (int)buf_length)
  {
    clientBuf[clientCount] = buf[bufferCount];
    clientCount++;
    bufferCount++;

    if(clientCount > 63)
    {
      dclient.write(clientBuf,64);
      clientCount = 0;
    }
  }

  if(clientCount > 0) dclient.write(clientBuf,clientCount);

  dclient.stop();
  if(Serial)
    Serial.println(F("Data disconnected"));

  if(!eRcv()) return 0;

  client.println(F("QUIT"));

  if(!eRcv()) return 0;

  client.stop();
  if(Serial)
    Serial.println(F("Command disconnected"));

  return 1;
}

byte eRcv()
{
  byte respCode;
  byte thisByte;

  while(!client.available()) delay(1);

  respCode = client.peek();

  outCount = 0;

  while(client.available())
  {  
    thisByte = client.read();
    if(Serial)
      Serial.write(thisByte);

    if(outCount < 127)
    {
      outBuf[outCount] = thisByte;
      outCount++;      
      outBuf[outCount] = 0;
    }
  }

  if(respCode >= '4')
  {
    efail();
    return 0;  
  }

  return 1;
}

void efail()
{
  byte thisByte = 0;

  client.println(F("QUIT"));

  while(!client.available()) delay(1);

  while(client.available())
  {  
    thisByte = client.read();
    if(Serial)
      Serial.write(thisByte);
  }

  client.stop();
  if(Serial)
    Serial.println(F("Command disconnected"));
}
