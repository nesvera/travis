#include "usb_communication.h"

#include <stdlib.h>

#define NUM_DATA_INPUT 3
#define NUM_DATA_OUTPUT 10

UsbCommunication::UsbCommunication(HardwareSerial &HWserial, uint32_t baudrate, uint32_t bufSize){

  pSerial = &HWserial;
  pSerial->begin(baudrate);

  buffer_size = bufSize;

  in_buffer = malloc(buffer_size * sizeof(char));
  memset (in_buffer, '\0', buffer_size);
  in_buffer_index = 0;

  new_data = false;  

  this->data_input = malloc(NUM_DATA_INPUT*sizeof(float));
}

void UsbCommunication::check(void){

  while( pSerial->available() > 0 ){
 
    char receivedChar = pSerial->read();

    if( receivedChar == '&' ){
      char_inicial = true;
      in_buffer_index = 0;
      
    }else if( receivedChar == '*' ){
      if( char_inicial == true ){
        in_buffer_index = 0;
        new_data = true;
        char_inicial = false;

        return;
        
      }else{
        in_buffer_index = 0;
        new_data = false;
        char_inicial = false;

      }

    }else{
      in_buffer[in_buffer_index] = receivedChar;
      in_buffer_index++;

    }    
  }
}

float * UsbCommunication::readMessage(void){

  char campo[16];
  int index_campo = 0;
  int index_data_input = 0;

  for( int i = 0 ; i < buffer_size ; i++ ){

    if( in_buffer[i] == '&' ){
      continue;
      
    }else if( in_buffer[i] == ';' ){
      
      float value = atof(campo);

      if( index_data_input < NUM_DATA_INPUT ){
        this->data_input[index_data_input] = value;
        index_data_input++;    
      }else{
        break;
      }

      memset(campo, '\0', sizeof(campo));
      index_campo = 0;
      
    }else if( in_buffer[i] == '*'){
      break;

    }else if( in_buffer[i] == '\0' ){
      break;
      
    }else{
      campo[index_campo] = in_buffer[i];
      index_campo++;
      
    }
    
  }
 
  new_data = false;
  memset (in_buffer, '\0', buffer_size);
  in_buffer_index = 0;

  return this->data_input;
}

bool UsbCommunication::hasNewData(void){
  return new_data;
}

void UsbCommunication::flush(void){
  pSerial->flush();
  while( pSerial->available() > 0 ){
    char receivedChar = pSerial->read();
  }
}

void UsbCommunication::writeMessage(float *message){

  String data_output;
  data_output = "&";

  int i;
  for(i=0; i<NUM_DATA_OUTPUT; i++){
    data_output = data_output + message[i] + ";";
  }

  data_output = data_output + "*\0";

  Serial.print(data_output);

  
}

