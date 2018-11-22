
/*#define DEBUG_LOG_VALUES*/

#define MAX_TX_BUFFER_SIZE 8

#define DEAD_ZONE_ESTIMATE 205



#ifdef DEBUG_LOG_VALUES
	#define LOG_BUFF_SIZE 512
	uint16_t pos_log_buffer[LOG_BUFF_SIZE];
	long int global_log_buffer[LOG_BUFF_SIZE];
	uint16_t speed_log_buffer[LOG_BUFF_SIZE];
	unsigned int buff_start = 0;
	unsigned int buff_end = 0;
#endif

// Dynamixel State (irectly read from dynamixel):
volatile uint8_t dyn_error;
volatile uint8_t dyn_error_sticky;
volatile uint16_t present_position;
volatile int16_t present_speed;
volatile int16_t present_load;
volatile uint8_t present_voltage;
volatile uint8_t present_temperature;
volatile uint16_t registered_instruction;
volatile uint8_t moving;

// Global position estimative, tries to account for full rotations:
volatile long int global_pos = 0;
volatile uint16_t last_position = 0;

struct tx_msg{
	unsigned char buff[16];
	int len;
	int sent;
	void (*callback)(char *);
};

// We allocate the queue for outgoing messages with 1 extra position
// This position will store th fallback command to send in case the queue is empty
volatile struct tx_msg tx_buffer[MAX_TX_BUFFER_SIZE + 1];
volatile int tx_buffer_start = 0;
volatile int tx_buffer_size = 0;

// Indicates there was a communication error, probably the first 2 read bytes were not 0xFF:
int transmission_error = 0;

// Buffer for received data:
unsigned char recv[32];
int recv_count = 0;
int expected_recv = 255;



enum Dynamixel_Instruction{
	DYN_RD = 0x02,
	DYN_WR = 0x03,
	DYN_RWR = 0x04,
	DYN_ACT = 0x05,
	DYN_PING = 0x01,
	DYN_RST = 0x06
};
int queueInstruction(unsigned char id, enum Dynamixel_Instruction inst, unsigned int n_args, ...);

volatile unsigned long int timeout_count;

ISR(TIMER3_COMPA_vect){
	if (recv_count != expected_recv){
		/*Serial.println("t");*/
		timeout_count++;
		tx_buffer[tx_buffer_start].sent = 0;
	}
	transmission_error = 0;
	recv_count = 0;
	expected_recv = 255;
	

	// Turn off reception, turn on transmission and enable UDR clear interrupt:
	UCSR3B &= ~(1 << 4);
	UCSR3B |=  (1 << 3);
	UCSR3B |=  (1 << 5);

	// Disable these interrupts:
	TIMSK3 &= ~0x02;
	
}

ISR(USART3_UDRE_vect){
	UDR3 = tx_buffer[tx_buffer_start].buff[tx_buffer[tx_buffer_start].sent];
	tx_buffer[tx_buffer_start].sent++;
	if (tx_buffer[tx_buffer_start].sent >= tx_buffer[tx_buffer_start].len){
		// turn off this interrupt:
		UCSR3B &= ~(1 << 5);
	}
}

ISR(USART3_RX_vect){
	recv[recv_count] = UDR3;
	// Set an interrupt to handle timeout if do not finish receiving all expected bytes to re-transmit the instruction:
	OCR3A = TCNT3 + 3560;// 160 uS
	TIMSK3 = TIMSK3 | 0x02;

	// If in error state, we just read the register to clear the interrupt, and quit:
	if (transmission_error){
		/*Serial.println("setou erro");*/
		return;}

	// Count one more received byte:
	recv_count++;
	// Check for errors and extract some info depending on position:
	if (recv_count <= 2){
		if (recv[recv_count - 1] != 0xFF){
			transmission_error = 1;
			// reset sent count, so we send this again:
			tx_buffer[tx_buffer_start].sent = 0;
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

		// Set an interrupt to begin next transmission
		OCR3A = TCNT3 + 1600;// about 100uS
		TIMSK3 |= (1 << 1);
	}

}

ISR(USART3_TX_vect){
	// turn off transmission, turn on reception:
	UCSR3B &= ~(1 << 3);
	UCSR3B |=  (1 << 4);

	//Set an interrupt in case dynamixel fails to respond. The time until the interrupt is calculated to be bigger than
	// the servo return delay time and some security margin:	
	OCR3A = TCNT3 + 32000;// about 1ms
	TIMSK3 |= 0x02;
}



/*
 * Function for adding an instruction to the buffer:
 * Usage:
 *   queueInstruction(id, inst, n_args, arg1, arg2, arg3 ..., argn, cb);
 * Parameters:
 *   id     : id of dynamixel
 *   int    : type of instruction to send
 *   n_args : the number of arguments for the instruction
 *   argn   : the n_args arguments to be sent
 *   cb     : the callback function to be executed at the end
 */
int queueInstruction(unsigned char id, enum Dynamixel_Instruction inst, unsigned int n_args, ...){
	// If buffer is full, quit with error:	
	if (tx_buffer_size >= MAX_TX_BUFFER_SIZE)
		return -1;

	// Find position to store next packet:
	int next_pos = (tx_buffer_start + tx_buffer_size) % MAX_TX_BUFFER_SIZE;
	if (tx_buffer_size == 0)
		next_pos = 0;
	/*Serial.print("adding instruction: ");*/
	/*Serial.print("pos: ");*/
	/*Serial.print(next_pos);*/
	/*Serial.print(" / size:");*/
	/*Serial.println(tx_buffer_size);*/

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
}

void telemetry_callback(unsigned char *recv){
	/*Serial.print("a");*/
	dyn_error = recv[4];
	dyn_error_sticky |= dyn_error;

	*((uint8_t *)&present_position) = recv[5];
	*(((uint8_t *)&present_position) + 1) = recv[6];

	if (last_position == 1023){
		if (present_position < 128){
			global_pos += DEAD_ZONE_ESTIMATE + (int)present_position;
			last_position = present_position;
		}else if(present_position > 900){
			global_pos -= 1023 - (int)present_position;
			last_position = present_position;
		}
	}else if(last_position == 0){
		if (present_position < 128 && present_position > 0){
			global_pos += (int)present_position;
			last_position = present_position;
		}else if(present_position > 900 && present_position < 1024){
			global_pos -= DEAD_ZONE_ESTIMATE + 1023 - (int)present_position;
			last_position = present_position;
		}
	}else{
		global_pos += (int)present_position - (int)last_position;
		last_position = present_position;
	}


	*((uint8_t *)&present_speed) = recv[7];
	*(((uint8_t *)&present_speed) + 1) = recv[8];
	if (present_speed & 0x400)
		present_speed = -(present_speed & 0x3FF);

	*((uint8_t *)&present_load) = recv[9];
	*(((uint8_t *)&present_load) + 1) = recv[10];
	if (present_load & 0x400)
		present_load = -(present_load & 0x3FF);

	// Log position values if enabled:
	#ifdef DEBUG_LOG_VALUES
		if ((buff_end + 1) % LOG_BUFF_SIZE != buff_start){
			pos_log_buffer[buff_end] = present_position;
			global_log_buffer[buff_end] = global_pos;
			speed_log_buffer[buff_end] = present_speed;
			buff_end = (buff_end + 1) % LOG_BUFF_SIZE;
		}else{
			pos_log_buffer[(buff_end + LOG_BUFF_SIZE - 1) % LOG_BUFF_SIZE] = 0xFFFF;
		}
	#endif
}

void status_callback(unsigned char *recv){
	/*Serial.println("b");*/
	dyn_error = recv[4];
	dyn_error_sticky |= dyn_error;
}

void setup() {
	Serial.begin(2000000);
	
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
	tx_buffer[8].callback = &telemetry_callback;
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

	/*TCCR3A &= ~0xC0;*/
	TCCR3A = 0;
	TCCR3B = 1;
	/*OCR3A = 600;*/
	// Disable timer interrupts:
	TIMSK3 &= ~0x02;

	// Wait until we get the current position:
	delay(500);
	global_pos = present_position;
	last_position = present_position;
	delay(500);

	queueInstruction(1, DYN_WR, 3, 8, 0, 0, status_callback);
	queueInstruction(1, DYN_WR, 3, 32, 50, 5, status_callback);
	queueInstruction(1, DYN_WR, 3, 34, 240, 1, status_callback);

	// start moving:
	/*queueInstruction(1, DYN_WR, 3, 30, 200, 0, status_callback);*/
}

unsigned long int last_action = millis();
int fez = 1;

void loop() {


	/*if (pose = NEUTRO)*/
		/*manda comando e espera chegar no 0 de posicao*/
	/*else if pose == esquerda vai pra la e fica verificando se chega no limite*/

	/*delay(200);*/

	#ifdef DEBUG_LOG_VALUES
		while(buff_start != buff_end){
			Serial.print("a");
			Serial.print(pos_log_buffer[buff_start]);
			Serial.print("\nb");
			Serial.print(global_log_buffer[buff_start]);
			Serial.print("\nc");
			Serial.print(speed_log_buffer[buff_start]);
			Serial.print("\n");
			buff_start = (buff_start + 1) % LOG_BUFF_SIZE;
		}
	#endif

	if (dyn_error){
		Serial.print("!!DYNAMIXEL ERROR!! :");
		Serial.println(dyn_error);
	}
	Serial.println("---");
	Serial.print("error count:");
	Serial.println(timeout_count);
	Serial.print("present position(raw):");
	Serial.println(present_position);
	Serial.print("global estimate:");
	Serial.println(global_pos);
	Serial.print("present speed:");
	Serial.println(present_speed);
	Serial.print("present load:");
	Serial.println(present_load);

	int16_t val = analogRead(A0);
	val -= 512;
	if (val < 0)
		val = -val | 0x400;
	if (val < 100)
		val = 0;
	queueInstruction(1, DYN_WR, 3, 32, val & 0xFF, (val & 0x700) >> 8, status_callback);
	delay(10);
}
