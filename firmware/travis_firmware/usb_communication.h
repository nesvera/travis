#ifndef usb_communication_h
#define usb_communication_h

#include <Arduino.h>

class UsbCommunication{

  public:

    UsbCommunication(HardwareSerial &HWserial, uint32_t baudrate, uint32_t buf_Size);

    void check(void);
    float *readMessage(void); 
    void writeMessage(float *message);
    bool hasNewData(void);
    void flush(void);
    

  private:

    HardwareSerial *pSerial;
    uint32_t baudrate;
    uint32_t buffer_size;

    bool new_data;

    char *in_buffer;
    uint32_t in_buffer_index;

    bool char_inicial;

    float *data_input;
    
};

#endif
