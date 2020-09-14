
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>


#include "pcm_sample.h"
#include "uart_lib.h"
#include <avr/interrupt.h>
#include <util/delay.h>

#define PCM_NOTE_A (1<<6) 
#define PCM_NOTE_B (1<<7)
#define PCM_NOTE_TEST 0

#define RECORD_SIZE 101

#define PAUSE 0
#define START 1

volatile unsigned int sample;
volatile int sample_count;
volatile int status=0;
volatile int pcm_turn_on_flag=0;
void PCM_off();
void PCM_on();
unsigned char scan_gpio_input_data();
unsigned char gpio_buffer;

volatile uint8_t buffer;
volatile char record_flag=0; 
volatile char mem_set_flag=0;
volatile char record_finish_flag=0;
volatile char play_flag=0;

uint8_t record_buff[RECORD_SIZE] = { 0, };

struct {
	
	volatile char record_tick_100ms;
	volatile int play_tick_100ms;
	int tick_10ms;
	volatile uint32_t ticks;
}TICK;


//실제 녹음 버튼
ISR(INT0_vect){
	record_flag=1; 
	TICK.tick_10ms=0; 
	PCM_on(PCM_NOTE_TEST);
	mem_set_flag=1;
	 
}
// 재생버튼
ISR(INT1_vect){
	play_flag=1;
}

ISR(USART0_RX_vect)
{
	while(!(UCSR0A & (1<<UDRE0)));
	buffer=UDR0;
}
ISR(TIMER0_COMP_vect)//TIMER0_OVF_vect 
{
	//count_pause(PAUSE);
	static unsigned char buff;
	//static char toggle=0;
	PORTC ^= 0x0f;
	
	/*좀 더 정확한 32khz 속도를 만들기 위해 (OCR0값이 62.5로 나왔을 경우..!)
	toggle^=0x01;
 	if(toggle) OCR0=62;
 	else  OCR0=61;
	*/
	TICK.ticks++;
	
    //TCNT0=256-16;
	if(pcm_turn_on_flag){
		sample_count--;
		if (sample_count == 0)
		{
			sample_count = 2;
			switch(status){
				case PCM_NOTE_B:
					buff=pgm_read_byte(&pcm_samples[sample++]);     
					OCR1A =buff;
					//OCR1B = buff;
					if(sample>pcm_b_length)PCM_off();
				
				break;
				case PCM_NOTE_A:
					buff=pgm_read_byte(&PCM_A[sample++]);
					OCR1A =buff;
					//OCR1B = buff;
					if(sample>pcm_a_length) PCM_off();
				break;
				case PCM_NOTE_TEST:
				buff=pgm_read_byte(&recordTEST[sample++]);
				OCR1A =buff;
				//OCR1B = buff;
				if(sample>pcm_test_length) PCM_off();
				break;
			
			}    
		
			//buff=test[sample++];
		
		
		
		}
	}//if(pcm_turn_on_flag) end
	
	
}


int main(void)
{
//unsigned char input;
	//A port : 건반용 채널
	DDRA=0x00; 
	PORTA=0xff; //내부 풀업 이용 PUD->0
	PORTD|=0x01; //인터럽트 핀 내부 풀업모드
	
	DDRC=0x0f;
	PORTC=0xFF;
	
	EIMSK=0x01; 	//0b 0000 0001 INT0
	EICRA=0x02; 	//0b 0000 0010 falling edge triggered

	
	uart_init(0,9600);
	
	//eeprom TEST
	/* use OC1A pin as output */
    DDRB |= ((1<<DDRB5)|(1<<DDRB6));

    /*
    * clear OC1A on compare match
    * set OC1A at BOTTOM, 비반전 모드
    * Fast PWM, 8bit
    */
    TCCR1A = (1<<COM1A1) | (1<<WGM10|(1<<COM1B1)|((1<<COM1B0)));
   
    /*
    * Fast PWM, 8bit
    * 분주비 : clk/1 = 8MHz
    * PWM 주파수 = 8MHz / (255 + 1) = 31.25kHz == 2MHz / 
    */
    TCCR1B = (1<<WGM12) | (1<<CS10);
   
    /* 초기 듀티비 50% */
    OCR1A = 0x80;
	OCR1B = 0x80;
   
    /* 타이머 카운터0 SETUP 
	
	* CTC Mode 
	* 
	*/
 
    TCCR0=((1<<CS01)|(0<<CS00)|(1<<WGM01)|(0<<WGM00)); //16/8MHz > 2MHz (256/4)  ///0.5MHz  > 32kHz
    //TCNT0=196;
    //TCNT0=256-15;
	OCR0=(125-1);
	
	//OCR0=128;
	//TIMSK|=(1<<TOIE0);
    TIMSK=(1<<OCIE0);
    sample_count = 2;
	sei(); //인터럽트 허용
	//16MHz >> 255  
	//cli();
	//OCR1A=0x80;
   while(1)
   {
	   
	   
			
		   //1. 임시 버퍼에 저장해둔 뒤,
		   //2. 녹음이 모두 끝났을 때
		   //3. 버퍼 값들을 플래쉬메모리에 저장 eeprom_update_byte((uint8_t*)record_tick_100ms,PCM_NOTE_B);
		   
		   //설마 녹음하는 과정에서 재생함수가 영향을 주나 ?>NO
		   
		   if(TICK.ticks%160==0){ // 10ms마다 
// 			   TICK.play_tick_10ms++;
// 			   TICK.record_tick_10ms++;
				TICK.tick_10ms++;			   
			    //녹음시 녹음 관련 tick 활성화
			   if(record_flag==1) {
				       if((TICK.tick_10ms%10==0)){//100ms마다 저장
							TICK.record_tick_100ms++; 
					      
					       if(TICK.record_tick_100ms>=100) {record_flag=0; record_finish_flag=1;}
				       }  
			   }else TICK.record_tick_100ms=0;
			   
			   
			   //녹음본 재생 시 관련 tick 활성화
			   if(play_flag==1) {
				   if(TICK.tick_10ms%10==0){
					    TICK.play_tick_100ms++;
					   static uint8_t eeprom_buff;
					  
					   //10초가 넘었을 때 ! 다시 알아서 녹음 기능을 비활성화 시킨다
					    eeprom_buff=eeprom_read_byte((const uint8_t*)TICK.play_tick_100ms);//play_tick_100ms
					    if(eeprom_buff) PCM_on(eeprom_buff);
					   if(TICK.play_tick_100ms>=100) play_flag=0;
						   
						   
				   }
			   }
			   else TICK.play_tick_100ms=0; 
			   
			   static unsigned char edge_detect=0;
			   
			   //키 입력 이벤트 처리
			   gpio_buffer=scan_gpio_input_data();
			   if(gpio_buffer){
				   if(edge_detect==0){
					   //엣지 검출 구문
					   edge_detect=1;
					   //uart0_tx_string("det\n");
					   	
					   PCM_on(PCM_NOTE_B);
					   //여기에 녹음한거 버퍼 저장
					   //record_buff[record_tick_100ms];
					   
					   gpio_buffer=scan_gpio_input_data();
					   //현재 이곳에 들어가는 버퍼의 경우, 단 하나 입력만 인정함. 동시 입력은 무시됨.
					   //즉, 순간적인 상황에서 조금이라도 먼저 입력된 버튼에 대해서 인식이 되고
					   //만일의 경우 두개가 아주 짧은 시간에 동시에 눌린 후에 스캔이 된다면, 이땐 아예 인식이 안됨.
					   switch(gpio_buffer){
						   
						   if(record_flag){
							   //버퍼에 note 기록
							   case PCM_NOTE_A: record_buff[(int)TICK.record_tick_100ms]=PCM_NOTE_A;break;
							   case PCM_NOTE_B: record_buff[(int)TICK.record_tick_100ms]=PCM_NOTE_B;break;
							   //PCM 음성 리스트 추가할 것
							}
					   }
					   
					   if(record_flag) {
						   
						}
				   }
			   }
			   else edge_detect=0;
			   
			  
			   
		   //uart0_tx_string(HexToString(scan_gpio_input_data()));
			     

			
			   
			   //uart0_tx_string(HexToString(scan_gpio_input_data()));
			      
		   }//if(TICK.ticks%160==0) end
		   
		   
		   //녹음이 종료되면 들어가는 구문
		   if(record_finish_flag)
		   {
			   
			   for(int i=0; i<RECORD_SIZE;i++) eeprom_update_byte((uint8_t*)i,record_buff[i]);   
			   
			   //이 딜레이는 아무 이유없이 넣은것이므로, 빼도 상관 없음.
			   PCM_on(0);
			   record_finish_flag=0;
			   //이건 노파심에 넣은거임. 있던 없던 동작에 영향을 주지 않을수 있음.
			   record_flag=0;
			   
		   }
		   
		   
		  // uart0_tx_string(IntToString(record_tick_100ms));
		   
		   if(mem_set_flag){
			   memset(record_buff,0,sizeof(record_buff));
			   mem_set_flag=0;
		   }
// 		   uart0_tx_string(HexToString(scan_gpio_input_data()));
// 		   uart0_tx_char('\n');
		   
		   
/***** dummy code. in terminal env.
		   uart0_tx_char(eeprom_read_byte((uint8_t*)0));00
 		   if(eeprom_read_byte((uint8_t*)0)=='a')
 		   PCM_on(PCM_NOTE_B);
 		   else if(eeprom_read_byte((uint8_t*)0)=='b'){
 			   PCM_off();
 		   }
 		   else PCM_on(PCM_NOTE_A);
*/



		   /*_delay_ms(200);
		   PCM_on(PCM_NOTE_B);
		   _delay_ms(200);
		   PCM_on(PCM_NOTE_A);
		   _delay_ms(200);
		   PCM_on(PCM_NOTE_B);
		   _delay_ms(200);*/
	
//    		input=PINC&0x01;
//		
// 		if(input==0)
// 			input=PINC&0x01;
// 		if(input==1)
// 		{
// 			sei();
// 			_delay_ms(2100);
// 			sample=0;
// 		}
// 		else
// 			cli();
		
   }

}

unsigned char scan_gpio_input_data(){
	unsigned char buff_=PINA;
	buff_=~(buff_); //모든비트 반전
	return buff_;
}

void PCM_off()
{
	sample=0;
	//TIMSK&=~(1<<TOIE0);
	
	//다른 tick들 때문에 함부로 끌수가 없다
	//TIMSK&=~(1<<OCIE0);
	pcm_turn_on_flag=0;
	
	OCR1A = 0x80;
	OCR1B = 0x80;
	
	//TCCR1A&=~(1<<WGM10);
}
void PCM_on(int note)
{
	
	status=note;
	sample=0;
	pcm_turn_on_flag=1;
	OCR1A = 0x80;
	OCR1B = 0x80;
	
	//TIMSK|=(1<<TOIE0);
	TCNT0=0;
	TIMSK|=(1<<OCIE0);
	TCCR1A|=(1<<WGM10);
	
}