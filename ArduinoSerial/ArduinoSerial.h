/*
 * ArduinoSerial.h
 *
 *  Created on: 04/08/2016
 *      Author: Iñaki Iriarte Muñoz
 */
#include "Arduino.h"
#ifndef ARDUINOSERIAL_H_
#define ARDUINOSERIAL_H_
 	#define MSG_RW 0
 	#define MSG_CFG 1

	#define PTY_INPUT 0
	#define PTY_OUTPUT 1
	#define PTY_PWM 2
	#define PTY_UNUSED 3

	#define BLINK_PIN 13
 	
 	#define READ  0
	#define WRITE  1
	
	#define ANALOG  0
	#define DIGITAL 1

	#define UNSET -1

	#define PIN_TYPE_MASK 1 << 5
	#define OPERATION_TYPE_MASK  1 << 4
	#define HI_WORD_BIT_MASK 0xF
	#define LOW_WORD_BIT_MASK 0x3F
	
	#define MIN_DIG_INPUT 0
	#define MAX_DIG_INPUT 63
	
	#define MIN_ANA_INPUT 0
	#define MAX_ANA_INPUT 63
	
	#define DIG_IN_CNT  1 + MAX_DIG_INPUT - MIN_DIG_INPUT
	#define ANA_IN_CNT  1 + MAX_ANA_INPUT - MIN_ANA_INPUT
  #define CONFIG_MESSAGE_MASK 0xC0
  #define CONFIG_PIN_TYPE_MASK  1 << 3
  #define CONFIG_PIN_FUNCTION_MASK  0x3

typedef struct Message{
	byte messageType;
	byte address;
	int value;
	byte operationType;
	byte pinType;
};



class ArduinoSerial {
private:
		int digPinValues[DIG_IN_CNT];
		int anaPinValues[ANA_IN_CNT];
		byte digPinState[DIG_IN_CNT];
		byte anaPinState[ANA_IN_CNT];
		int bytebuffer[3];
		byte readBuffer[3];
		byte buffer[3];		

		Message msg;
		bool isMessageComplete();
		void initPinValues();
		byte* encode(Message* msg, byte* ret);
		Message* decode(byte* data, Message* ret);
		void readInput(Message* ret, byte pinType, byte pinNumber);
		void readAndSendPinValues(byte pinType, byte minPin,byte maxPin, int* values);
		void executeMessage(Message* msg);
		byte getPinState(byte pinType, byte address);
		void configPin(byte pinType, byte address, byte state);
		void setPinState(byte pinType, byte address, byte state);
public:
	ArduinoSerial();
	void serialEvent();
	void readInputs();
	virtual ~ArduinoSerial();
};

#endif /* ARDUINOSERIAL_H_ */
