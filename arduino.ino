
#define MAX_TX_BUFFER_SIZE 8

uint16_t present_position;
uint16_t present_speed;
uint16_t present_load;
uint8_t present_voltage;
uint8_t present_temperature;
uint16_t registered_instruction;
uint8_t moving;

struct tx_msg{
	unsigned char buff[16];
	int len;
	int sent;
	void (*callback)(char *);
};

struct tx_msg tx_buffer[9];
int tx_buffer_start = 0;
int tx_buffer_size = 0;

unsigned long int last_received_byte = 0;

int error = 0;

unsigned char recv[32];
int recv_count = 0;
int expected_recv = 255;



enum bob{
	DYN_RD = 0x02,
	DYN_WR = 0x03,
	DYN_RWR = 0x04,
	DYN_ACT = 0x05,
	DYN_PING = 0x01,
	DYN_RST = 0x06
};
int queueInstruction(unsigned char id, enum bob inst, unsigned int n_args, ...);


ISR(USART3_UDRE_vect){
	if (tx_buffer[tx_buffer_start].sent < tx_buffer[tx_buffer_start].len){
		UDR3 = tx_buffer[tx_buffer_start].buff[tx_buffer[tx_buffer_start].sent];
		tx_buffer[tx_buffer_start].sent++;
	}else{
		// turn off this interrupt:
		UCSR3B &= ~(1 << 5);
	}
	/*Serial.print(tx_buffer_start);*/
	/*Serial.print("-");*/
	/*Serial.print(tx_buffer[tx_buffer_start].sent);*/
	/*Serial.print("-");*/
	/*Serial.print(tx_buffer[tx_buffer_start].len);*/
	/*Serial.print("-");*/
	/*Serial.println(tx_buffer_size);*/
}

ISR(USART3_RX_vect){
	recv[recv_count] = UDR3;
	/*Serial.println((unsigned char)recv[recv_count]);*/
	last_received_byte = millis();
	// If in error state, we just read the register to clear the interrupt, and quit:
	if (error){
		/*Serial.println("setou erro");*/
		return;}

	// Count one more received byte:
	recv_count++;
	// Check for errors and extract some info depending on position:
	if (recv_count <= 2){
		if (recv[recv_count - 1] != 0xFF){
			error = 1;
		}
	}else if(recv_count == 4){
		expected_recv = recv[3] + 4;
		/*Serial.print("expected recv:");*/
		/*Serial.println(expected_recv);*/
	}else if(recv_count == expected_recv){
		unsigned char expected_crc = 0;
		for (int i = 2; i < recv_count - 1; i++){
			expected_crc += recv[i];
		}
		expected_crc = ~expected_crc;
		if (expected_crc == recv[recv_count - 1]){
			if (tx_buffer[tx_buffer_start].callback)
				(*tx_buffer[tx_buffer_start].callback)(recv);
		}else{
			/*Serial.println("c");*/
		}

		buffer_step();

		// Turn off reception, turn on transmission and enable UDR clear interrupt:
		UCSR3B &= ~(1 << 4);
		UCSR3B |=  (1 << 3);
		UCSR3B |=  (1 << 5);

	}

}

ISR(USART3_TX_vect){
	/*Serial.println("g");*/
	last_received_byte = millis();
	// turn off transmission, turn on reception:
	UCSR3B &= ~(1 << 3);
	UCSR3B |=  (1 << 4);
}



/*
 * Function for adding an instruction to the buffer:
 * Usage:
 *   queueInstruction(id, inst, n_args, arg1, arg2, arg3 ..., argn, cb);
 * Parameters:
 *   id : id of dynamixel
 *   int: type of instruction to send
 *   n_args: the number of arguments for the instruction
 *   argn : the n_args arguments to be sent
 *   cb   : the callback function to be executed at the end
 */
int queueInstruction(unsigned char id, enum bob inst, unsigned int n_args, ...){
	// If buffer is full, quit with error:	
	if (tx_buffer_size >= MAX_TX_BUFFER_SIZE)
		return -1;

	// Find position to store next packet:
	int next_pos = (tx_buffer_start + 1) % MAX_TX_BUFFER_SIZE;
	if (tx_buffer_start == 8)
		next_pos = 0;

	va_list valist;
	va_start(valist, n_args + 1);
	
	// Fill buffer entry:
	tx_buffer[next_pos].buff[0] = 0xFF;
	tx_buffer[next_pos].buff[1] = 0xFF;
	tx_buffer[next_pos].buff[2] = id;
	tx_buffer[next_pos].buff[3] = n_args + 2;
	tx_buffer[next_pos].buff[4] = inst;
	unsigned char sum = id + n_args + 2 + inst;
	for (int i = 0; i < n_args; i++){
		tx_buffer[next_pos].buff[i + 5] = va_arg(valist, int);
		sum += tx_buffer[next_pos].buff[i + 5];
	}
	// calculate CRC:
	tx_buffer[next_pos].buff[5 + n_args] = ~sum;

	tx_buffer[next_pos].len = 6 + n_args;
	tx_buffer[next_pos].sent = 0;
	tx_buffer[next_pos].callback = va_arg(valist, void(*)(unsigned char *));
	// signal that we have one more element waiting in the buffer:
	tx_buffer_size++;
	// free argument list:
	va_end(valist);
	
}

void buffer_step(){
	/*Serial.print(tx_buffer_size);*/
	/*Serial.print("buffer clear");*/
	/*Serial.println(tx_buffer_start);*/

	if (tx_buffer_size == 0){
		tx_buffer_start = 8;
		tx_buffer[8].sent = 0;
	}else{
		if (tx_buffer_start == 8){
			tx_buffer_start = 0;
		}else{
			tx_buffer_start = (tx_buffer_start + 1) % MAX_TX_BUFFER_SIZE;
			tx_buffer_size--;

			if (tx_buffer_size == 0){
				tx_buffer_start = 8;
				tx_buffer[8].sent = 0;
			}
		}
	}
	recv_count = 0;
	expected_recv = 255;

}



void status_callback(unsigned char *recv){
	/*for (int i = 0; i < recv[3] + 4; i++){*/
		/*Serial.println(recv[i]);*/
	/*}*/
	/*Serial.println("#");*/
	/*present_position = *((uint16_t *)(recv + 5));*/
	/*present_speed = *((uint16_t *)(recv + 7));*/
	/*present_load = *((uint16_t *)(recv + 9));*/
	/*Serial.println("ok");*/
	/*Serial.println(recv[4]);*/



	/**((uint8_t *)present_position) = *((uint16_t *)recv + 1);*/
	/**((uint8_t *)(present_position + 1)) = *((uint16_t *)recv);*/
	/**((uint8_t *)present_speed) = *((uint16_t *)recv + 3);*/
	/**((uint8_t *)(present_speed + 1)) = *((uint16_t *)recv + 2);*/
	/**((uint8_t *)present_load) = *((uint16_t *)recv + 5);*/
	/**((uint8_t *)(present_load + 1)) = *((uint16_t *)recv + 4);*/

}

void setup() {
	Serial.begin(1000000);
	pinMode(A0, INPUT);

	/*Serial3.write(0xFF);*/
	/*Serial3.write(0xFF);*/
	/*Serial3.write(0x01);*/
	/*Serial3.write(0x05);//len*/
	/*Serial3.write(0x03);//inst*/
	/*Serial3.write(30);//ende*/
	/*Serial3.write(pos & 0xFF);//dat*/
	/*Serial3.write(pos >> 8);//dat*/
	/*Serial3.write((byte)((~(0x01 + 0x05 + 0x03 + 30 + (pos >> 8) + (pos & 0xFF))) & 0xFF));*/
	
	
	tx_buffer[8].buff[0] = 0xFF;
	tx_buffer[8].buff[1] = 0xFF;
	tx_buffer[8].buff[2] = 0x01;
	tx_buffer[8].buff[3] = 0x04;
	tx_buffer[8].buff[4] = 0x02;
	tx_buffer[8].buff[5] = 36;
	tx_buffer[8].buff[6] = 11;
	tx_buffer[8].buff[7] = ~(0x01 + 0x04 + 0x02 + 36 + 11);
	tx_buffer[8].len = 8;
	tx_buffer[8].sent = 0;
	tx_buffer[8].callback = &status_callback;

	/*aSpeed=-aSpeed | 1024;*/


	/*tx_buffer[8].buff[0] = 0xFF;*/
	/*tx_buffer[8].buff[1] = 0xFF;*/
	/*tx_buffer[8].buff[2] = 0x01;*/
	/*tx_buffer[8].buff[3] = 5;*/
	/*tx_buffer[8].buff[4] = 0x03;*/
	/*tx_buffer[8].buff[5] = 32;*/
	/*tx_buffer[8].buff[6] = 240;*/
	/*tx_buffer[8].buff[7] = 0;*/
	/*tx_buffer[8].buff[8] = ~(0x01 + 0x05 + 0x03 + 32 + 240 + 0);*/
	/*tx_buffer[8].len = 9;*/
	/*tx_buffer[8].sent = 0;*/
	/*tx_buffer[8].callback = status_callback;*/


	/*tx_buffer[8].buff[0] = 0xFF;*/
	/*tx_buffer[8].buff[1] = 0xFF;*/
	/*tx_buffer[8].buff[2] = 0x01;*/
	/*tx_buffer[8].buff[3] = 4;*/
	/*tx_buffer[8].buff[4] = 0x03;*/
	/*tx_buffer[8].buff[5] = 24;*/
	/*tx_buffer[8].buff[6] = 1;*/
	/*tx_buffer[8].buff[8] = ~(0x01 + 0x04 + 0x03 + 24 + 1);*/
	/*tx_buffer[8].len = 9;*/
	/*tx_buffer[8].sent = 0;*/
	/*tx_buffer[8].callback = status_callback;*/
	tx_buffer_start = 8;
	tx_buffer_size = 0;


	tx_buffer_start = 8;
	tx_buffer_size = 0;

	int ubrr = 9;
	UBRR3H = (unsigned char)(ubrr>>8);
	UBRR3L = (unsigned char)ubrr;

	// Enable reception interruption:
	UCSR3B |= 1 << 7;
	// Enable transmit complete interruption:
	UCSR3B |= 1 << 6;

	
	// Turn off reception, turn on transmission and enable UDR clear interrupt:
	UCSR3B &= ~(1 << 4);
	UCSR3B |=  (1 << 3);
	UCSR3B |=  (1 << 5);


}

int fez = 0;
void loop() {
	if ( (UCSR3B & (1 << 4)) && (millis() - last_received_byte >= 10)){
		last_received_byte = millis();
		Serial.print(micros());
		Serial.println("timeout");
		if (tx_buffer_size == 0){
			tx_buffer_start = 8;
			tx_buffer[8].sent = 0;
		}else{
			if (tx_buffer_start == 8){
				tx_buffer_start = 0;
			}else{
				tx_buffer_start = (tx_buffer_start + 1) % MAX_TX_BUFFER_SIZE;
				tx_buffer_size--;
			}
		}
		error = 0;
		recv_count = 0;
		expected_recv = 255;
		

		// Turn off reception, turn on transmission and enable UDR clear interrupt:
		UCSR3B &= ~(1 << 4);
		UCSR3B |=  (1 << 3);
		UCSR3B |=  (1 << 5);
	}
	if (error){
		Serial.print(micros());
		Serial.println("error");
		delay(50);
		buffer_step();
		error = 0;
		

		// Turn off reception, turn on transmission and enable UDR clear interrupt:
		UCSR3B &= ~(1 << 4);
		UCSR3B |=  (1 << 3);
		UCSR3B |=  (1 << 5);
	}

	/*Serial.print("Present position: ");*/
	/*Serial.println(present_position);*/
	/*Serial.print("Present speed: ");*/
	/*Serial.println(present_speed);*/
	/*Serial.print("Present load: ");*/
	/*Serial.println(present_load);*/
	/*Serial.print("Present voltage: ");*/
	/*Serial.println(present_voltage);*/
	/*Serial.print("Present temperature: ");*/
	/*Serial.println(present_temperature);*/
	/*Serial.print("Registered instruction: ");*/
	/*Serial.println(registered_instruction);*/
	/*Serial.print("Moving: ");*/
	/*Serial.println(moving);*/
	/*Serial.println("*******************************");*/

	if (millis() > 2000 && !fez){
		fez = 1;
		queueInstruction(1, DYN_WR, 3, 32, 240, 4, status_callback);
	}

	/*delay(1700);*/
	/*tx_buffer[8].buff[0] = 0xFF;*/
	/*tx_buffer[8].buff[1] = 0xFF;*/
	/*tx_buffer[8].buff[2] = 0x01;*/
	/*tx_buffer[8].buff[3] = 0x05;*/
	/*tx_buffer[8].buff[4] = 0x03;*/
	/*tx_buffer[8].buff[5] = 32;*/
	/*tx_buffer[8].buff[6] = 0xFF;*/
	/*tx_buffer[8].buff[7] = 10;*/
	/*tx_buffer[8].buff[8] = ~(0x01 + 0x05 + 0x03 + 32 + 0xFF + 10);*/
	/*tx_buffer[8].len = 9;*/
	/*tx_buffer[8].sent = 0;*/
	/*tx_buffer[8].callback = NULL;*/
	/*[>tx_buffer_size++;<]*/

	/*delay(1700);*/
}
