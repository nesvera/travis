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
}

void UsbCommunication::check(void){

  int input_str_size = 0;
  String input_str;
  
  while(input_str_size < 3){
    input_str = Serial.readStringUntil('\n');
    input_str_size = input_str.length();
  }

  int i;
  for(i=0 ; i<input_str_size; i++){ 
    char receivedChar = input_str[i];
  }

  new_data = true;
  return;
  
  for(i=0 ; i<input_str_size; i++){ 
    char receivedChar = input_str[i];

    if( receivedChar == '&' ){
      char_inicial = true;
      in_buffer_index = 0;

      //Serial.println("1");
      
    }else if( receivedChar == '*' ){
      if( char_inicial == true ){
        in_buffer_index = 0;
        new_data = true;
        char_inicial = false;

        //Serial.println("2");
        
        return;
        
      }else{
        in_buffer_index = 0;
        new_data = false;
        char_inicial = false;

        //Serial.println("3");
      }

    }else{
      in_buffer[in_buffer_index] = receivedChar;
      in_buffer_index++;

      //Serial.println("4 ");
    }    
  }
}

/*
void UsbCommunication::check(void){

  while( pSerial->available() > 0 ){
 
    char receivedChar = pSerial->read();

    if( receivedChar == '&' ){
      char_inicial = true;
      in_buffer_index = 0;

      Serial.println("1");
      
    }else if( receivedChar == '*' ){
      if( char_inicial == true ){
        in_buffer_index = 0;
        new_data = true;
        char_inicial = false;

        Serial.println("2");
        
        return;
        
      }else{
        in_buffer_index = 0;
        new_data = false;
        char_inicial = false;

        Serial.println("3");
      }

    }else{
      in_buffer[in_buffer_index] = receivedChar;
      in_buffer_index++;

      Serial.println("4 ");
    }    
  }
}

void UsbCommunication::check(void){

  bool char_inicial = false;
  bool char_final = false;
 
  while( pSerial->available() > 0 ){
 
    char receivedChar = pSerial->read();
    in_buffer[in_buffer_index] = receivedChar;
    in_buffer_index++;

    if( receivedChar == '*' ){

      // conferir se a mensagem esta correta
      int i;
      for(i=0; i<buffer_size; i++){

        if(in_buffer[i] == '&'){
          if(char_inicial == false){
            char_inicial = true;

            //Serial.println("1");
          }else{
            // mensagem com problemas
            memset (in_buffer, '\0', buffer_size);
            in_buffer_index = 0;

            //serialFlush();

            //Serial.println("2");
          }
          
        }else if(in_buffer[i] == '*'){
          if(char_inicial == false){
            // mensagem com problemas
            memset (in_buffer, '\0', buffer_size);
            in_buffer_index = 0;

            //serialFlush();

            //Serial.println("3");
          }else{
            new_data = true;

            //Serial.println("4");

          }
        }
      } 
    }
  }    
}
*/

float * UsbCommunication::readMessage(void){

  char campo[16];
  int index_campo = 0;
  int index_data_input = 0;

  float *data_input = malloc(NUM_DATA_INPUT*sizeof(float));

  for( int i = 0 ; i < buffer_size ; i++ ){

    if( in_buffer[i] == '&' ){
      continue;
      
    }else if( in_buffer[i] == ';' ){
      
      float value = atof(campo);

      if( index_data_input < NUM_DATA_INPUT ){
        data_input[index_data_input] = value;
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

  return data_input;
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

  data_output = data_output + "*\n";

  Serial.println(data_output);
}

