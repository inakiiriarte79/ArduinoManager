//============================================================================
// Name        : ArduinoSerial.cpp
// Author      : Iñaki Iriarte
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "Arduino.h"
#include "ArduinoSerial.h"
bool isConfigMessage(byte b){
	return (b  & CONFIG_MESSAGE_MASK) == CONFIG_MESSAGE_MASK;
}
ArduinoSerial::ArduinoSerial() {
	initPinValues();
	Serial.begin(9600);
}

ArduinoSerial::~ArduinoSerial() {
	// TODO Auto-generated destructor stub
}

void blink(int time){
			digitalWrite(BLINK_PIN, HIGH);
			delay(time);
			digitalWrite(BLINK_PIN, LOW);
}

void ArduinoSerial::initPinValues(){
			for(int i=0;i <= DIG_IN_CNT ; i++)
				digPinValues[i] = UNSET;
			for(int i=0;i <= ANA_IN_CNT; i++)
				anaPinValues[i] = UNSET;				
			for(int i=0;i <= ANA_IN_CNT; i++)
				anaPinState[i] = PTY_UNUSED;	
			for(int i=0;i <= DIG_IN_CNT; i++)
				digPinState[i] = PTY_UNUSED;						

	}
byte* ArduinoSerial::encode(Message* msg, byte* ret)
	{
		if(msg->messageType == MSG_RW)
		{
			ret[0] = msg->address;
			ret[1] = (byte) (
				0x40 |
				(msg->pinType == DIGITAL ? PIN_TYPE_MASK : 0) |
				(msg->operationType == WRITE ? OPERATION_TYPE_MASK : 0) |
				(msg->value >> 6)
			);
			ret[2] = (byte) (0x80 | (msg->value & LOW_WORD_BIT_MASK));			
		}else{
			 ret[0]= 0xC0 | (msg->address>>3)| (msg->pinType == DIGITAL ? CONFIG_PIN_TYPE_MASK : 0) | (msg->operationType == WRITE ? OPERATION_TYPE_MASK : 0);
			 ret[1]= 0xE0 | ((msg->address & 0x7) << 2) | msg->value;
		}
		return ret;	
	}

Message* ArduinoSerial::decode(byte* data, Message* ret)
	{	
		if(isConfigMessage(data[0]))			
		{
			ret->address = (byte)(((data[0]& 7) << 3) | ((data[1]>>2) & 0x7));
            ret->pinType = ((int)data[0] & CONFIG_PIN_TYPE_MASK) == 0 ? ANALOG : DIGITAL;
            ret->operationType = ((int)data[0] & OPERATION_TYPE_MASK) == 0 ? READ : WRITE;
            ret->value = data[1] & CONFIG_PIN_FUNCTION_MASK;
            ret->messageType = MSG_CFG;

		}else	
		{	
			ret->address = data[0];
			ret->pinType = ((int) data[1] & PIN_TYPE_MASK) == 0 ? ANALOG : DIGITAL;
			ret->operationType = ((int) data[1] & OPERATION_TYPE_MASK) == 0 ? READ : WRITE;
			ret->value = (((data[1] & HI_WORD_BIT_MASK) << 6) | (data[2] & LOW_WORD_BIT_MASK));
			ret->messageType = MSG_RW;
		}
		return ret;
	}

void ArduinoSerial::readInput(Message* ret, byte pinType, byte pinNumber) {
		ret->address = pinNumber;
		ret->operationType = READ;
		ret->pinType = pinType;
		ret->value = pinType == ANALOG ? analogRead(pinNumber) : digitalRead(pinNumber);
		ret->messageType = MSG_RW;
	}

void ArduinoSerial::readAndSendPinValues(byte pinType, byte minPin,byte maxPin, int* values){

		for(byte inputPin = minPin; inputPin <= maxPin; inputPin++){
			byte pinState = getPinState(pinType, inputPin);
			if(pinState == INPUT){
				readInput(&msg, pinType, inputPin);
				int idx = inputPin - minPin;
				if(values[idx] != msg.value){
					values[idx] = msg.value;
					encode(&msg, buffer);
					//Serial.print("Pin ");Serial.print(inputPin);Serial.print("=");Serial.println(msg.value);
					Serial.write(buffer, msg.messageType == MSG_RW ? 3 : 2);
				}
			}
		}
	}

bool ArduinoSerial::isMessageComplete()
	{
		return (isConfigMessage(readBuffer[0]) && readBuffer[1] ) || (readBuffer[1] && readBuffer[2]) ;
	}

void ArduinoSerial::executeMessage(Message* msg){
	if(msg->messageType == MSG_RW){
		if(msg->operationType == WRITE){
			if(msg->pinType == DIGITAL)
				digitalWrite(msg->address, msg->value);
			else
				analogWrite(msg->address, msg->value);
		}
	}else if(msg->messageType == MSG_CFG){
		configPin(msg->pinType, msg->address, msg->value);
	}

}

void ArduinoSerial::readInputs() {

		readAndSendPinValues(DIGITAL, MIN_DIG_INPUT, MAX_DIG_INPUT, digPinValues);
		readAndSendPinValues(ANALOG, MIN_ANA_INPUT, MAX_ANA_INPUT, anaPinValues);
}

   int getIndex(byte b) { return isConfigMessage(b) ? b >> 5 & 1 : b >> 6 & 3; }



void ArduinoSerial::serialEvent() {

		while (Serial.available()>0) {
			// get the new byte:
			byte b = (byte) Serial.read();
			readBuffer[getIndex(b)] = b;
			if (isMessageComplete())
			{
				decode(readBuffer, &msg);
				executeMessage(&msg);
				readBuffer[0] = readBuffer[1] = readBuffer[2] = 0;
			}
		}
	}

	byte ArduinoSerial::getPinState(byte pinType, byte address){
		if(pinType == ANALOG)
			return anaPinState[address];
		else
			return digPinState[address];
	}

	void ArduinoSerial::setPinState(byte pinType, byte address, byte state){
		if(pinType == ANALOG){
			anaPinState[address] = state;
		}else
			digPinState[address] = state;
	}

void ArduinoSerial::configPin(byte pinType, byte address, byte state){
		switch(state)
		{
			case PTY_UNUSED:
			break;
			case PTY_INPUT:
				pinMode(address, INPUT);
			break;
			case PTY_OUTPUT:
				pinMode(address, OUTPUT);
			break;
			case PTY_PWM:
			break;
		}
		setPinState(pinType, address, state);
}
