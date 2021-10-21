#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>

#define FORWARD true
#define BACKWARDS false
#define PI 3.14159265358979323846
//RAIO_RODA - m
#define RAIO_RODA 0.05 
//D_RODA_CENTROIDE - m
#define D_RODA_CENTROIDE 0.075

float tensao_bateria = 4; //Volts
float perdas = 0.94; //V
float max_tensao_motor = tensao_bateria - perdas;
float temperaturaADC = 25;
bool read_temp_sensor = false;
float tempC = 25;
float gasADC = 0;

float n_gas_0 = 0;
float n_gas_90 = 0;
float n_gas_180 = 0;
float n_gas_level = 0;

float luzADC = 0;
float n_luz = 0;
float tempo_atual = 0;
float tol_ang_1 = 2;
float tol_ang_2 = 0.01;

float tol_dist_1 = 0.02;
float tol_dist_2 = 0.005;

bool motor_ligado = false;
float interruption_interval = 0.001;//segundos
double total_time = 0;//segundos
float duty_atual = 0;
float rotacao_acumulada = 0;
float vel_giro = 0;
float delta_angle = 0;
float delta_angle_graus = 0;
float vel_linear_robo = 0;
float delta_vel = 0;
float angle_robo_rad = 0;

bool toggle_leds = false;
bool timer_interruption_flag = false;
uint32_t count_print = 0;
uint32_t count_total = 0;
uint16_t count_led = 0;
char buffer_int[40];
char buffer_float[40];
char temp_string[20];
char gas0_string[20];
char gas90_string[20];
char gas180_string[20];
char luz_string[30];

unsigned char switch_usart;

/* 
	1 -> p0 para p1
    2 -> p1 para p2
    3 -> p2 para p3
    4 -> p3 para p4
    5 -> p4 para p5
*/

int trajeto = 1;
/* 
	0 -> parado
	1 -> alinhando com ponto
    2 -> coletando dados
    3 -> movendo para ponto
    4 -> coleta ultimo ponto
    5 -> comemoracao
*/
int estado_robo =0;

int count_timer1 = 0;

struct Ponto_coleta{
	float x;
  	float y;
};

struct Robo{
	float x;
  	float y;
  	float angle;//em graus
  	int sentido_giro;// 0 = parado, 1= antihorario, 2= horario
	int sentido_robo;// 0 = parado, 1= pra frente, 2= pra tras
};

struct Ponto_coleta criar_ponto_coleta(float _x, float _y){
	Ponto_coleta ponto;
  	ponto.x = _x;
  	ponto.y = _y;
	return ponto;
}

struct Robo criar_robo(float _x, float _y, float _angle){
	Robo robo;
  	robo.x = _x;
  	robo.y = _y;
  	robo.angle = _angle;
  	robo.sentido_giro = 0;
  	robo.sentido_robo = 0;
  	return robo;
}

float calculate_angle(struct Robo robo, struct Ponto_coleta ponto){//computar o angulo para alinhar o robo com o ponto
	float delta_y = ponto.y - robo.y;
  	float delta_x = ponto.x - robo.x;
  	float angle = atan2(delta_y, delta_x); //angulo em radiano
  	float angle_graus = (angle*180)/PI;
  	return angle_graus;
}

float calculate_distance(struct Robo robo,struct Ponto_coleta ponto){
	float delta_y = ponto.y - robo.y;
  	float delta_x = ponto.x - robo.x;	
  	return sqrt(delta_y*delta_y + delta_x*delta_x);//distancia euclideana entre robo e o ponto
}
/*
Ponto_coleta p1 = criar_ponto_coleta(0, 100);
Ponto_coleta p2 = criar_ponto_coleta(50, 100);
Ponto_coleta p3 = criar_ponto_coleta(50, 250);
Ponto_coleta p4 = criar_ponto_coleta(150, 250);
Ponto_coleta p5 = criar_ponto_coleta(150, 0);
*/

Ponto_coleta p1 = criar_ponto_coleta(0, 0.5);
Ponto_coleta p2 = criar_ponto_coleta(0.5, 0.5);
Ponto_coleta p3 = criar_ponto_coleta(0.5, 1);
Ponto_coleta p4 = criar_ponto_coleta(1, 1);
Ponto_coleta p5 = criar_ponto_coleta(1, 0);

Robo robo = criar_robo(0,0,0);

Ponto_coleta pegar_ponto_trajeto(int trajeto){
  if(trajeto == 1){
    return p1;
  }
  if(trajeto == 2){
    return p2;
  }
  if(trajeto == 3){
    return p3;
  }
  if(trajeto == 4){
    return p4;
  }
  if(trajeto == 5){
    return p5;
  }
}

float pegar_duty_trajeto(int trajeto){
  if(trajeto == 1){
    return 75;
  }
  if(trajeto == 2){
    return 50;
  }
  if(trajeto == 3){
    return 25;
  }
  if(trajeto == 4){
    return 50;
  }
  if(trajeto == 5){
    return 100;
  }else{
    return 0;
  }
}

void set_red_led(bool status){
  if (status == true){
  	PORTD |= 0b00000100; //liga led vermelho q ta na porta A2	
  }else{
  	PORTD &= 0b11111011; //apaga led vermelho	
  }
}

void set_green_led(bool status){
  if (status == true){
  	PORTD |= 0b00001000; //liga led verde q ta na porta A3	
  }else{
  	PORTD &= 0b11110111; //apaga led verde	
  }
}

bool is_on(){
  int value = (PINC & 0b00100000);//valor de entrada q vem do switch p/ PC5 ta no reg PINC 2^5=32
  if (value == 32){
    return true;
  }else{
    return false;
  }
}

void spin_motor1_clockwise(){//combinacao 10
	PORTB |= 0b00100000;//saida PB5(D13) é setado em alta ->input1 do l293D
  	PORTB &= 0b11101111;//saida PB4(D12) é setado em baixa->input2 do l293D
}

void spin_motor1_counterclockwise(){//combinacao 01
	PORTB |= 0b00010000;//saida PB4(D12) é setado em alta->input2 do l293D
  	PORTB &= 0b11011111;//saida PB5(D13) é setado em baixa->input1 do l293D
}

void stop_motor1(){//combinacao 00
	PORTB &= 0b11001111;//saida PB4(D12) e PB5(D13) em baixa
}

void set_motor1(bool on, bool forward){
  if(on == true){//so havera movimento se o robo tiver on
    if(forward == true){
    	spin_motor1_clockwise();//gira motor1 no sentido horario
    }else{
    	spin_motor1_counterclockwise();	//gira motor1 no sentido antihorario
    }
  }else{
  	stop_motor1();//nao rotaciona motor se o robo estiver off	
  }  
}

void spin_motor2_clockwise(){//combinacao 10
	PORTB |= 0b00000001;//saida PB0(D8) é setado em alta->input3 do l293D
  	PORTB &= 0b11111011;//saida PB2(D10) é setado em baixa->input4 do l293D
}

void spin_motor2_counterclockwise(){//combinacao 01
	PORTB |= 0b00000100;//saida PB2(D10) é setado em alta->input4 do l293D
  	PORTB &= 0b11111110;//saida PB0(D8) é setado em baixa->input3 do l293D
}

void stop_motor2(){//combinacao 00
	PORTB &= 0b11111010;
}

void set_motor2(bool on, bool forward){
  if(on == true){//so havera movimento de motor se o robo tiver on
    if(forward == true){
    	spin_motor2_clockwise();//gira motor2 no sentido horario
    }else{
    	spin_motor2_counterclockwise();	//gira motor2 no sentido antihorario
    }
  }else{
  	stop_motor2();//nao rotaciona motor se o robo estiver off 	
  }  
}

void stop_motors(){
	stop_motor1();
    stop_motor2();
}

void spin_robot(){//para fazer a rotacao do robo -> motores giram em sentidos opostos
	set_motor1(true, FORWARD);
	set_motor2(true, BACKWARDS);
}

void spin_robot_ms(float time_ms){//fazendo rotacao baseado em tempo, em seguida para robo
	spin_robot();
  	_delay_ms(time_ms);
  	stop_motor1();
  	stop_motor2();
} 

void shutdown_robot(){
	stop_motor1();
  	stop_motor2();  
  	set_red_led(false);
  	set_green_led(false);
}

void init_pwm8bits(){//usa contador 0 - que é 8bits
  TCCR0A = 0b10000011;//modo fast pwm (ultimos 2 bits 1) / modo nao inversor (primeiros bits 10)
  TCCR0B = 0b00000011;	//escalonando por 64, fico com freq de 250kHz
}

void init_pwm8bits_servo(){//usa contador 0 - que é 8bits
  TCCR2A = 0b10000011;//modo fast pwm (ultimos 2 bits 1) / modo nao inversor (primeiros bits 10)
  TCCR2B = 0b00000111;	//escalonando por 64, fico com freq de 250kHz
}

void set_comp_reg_timer2(int value)
{
	OCR2A = value;
}

void servo_angle_0()
{
	set_comp_reg_timer2(0);
}

void servo_angle_90()
{
	set_comp_reg_timer2(24);
}

void servo_angle_180()
{
	set_comp_reg_timer2(50);
}

void set_pwm8bits_duty_cycle(float duty){
  OCR0A = (int)( (duty/100.0)*255.0 );//valor de duty cycle de 0 - 100 * valor da contagem (2^8 = 256, entao tem 255 valores contados)
}

float get_rpm(float duty){//a rpm vai variar para cada percursso, pq a tensao enviada pro motor vai mudar (usando o pwm) 
  float rpm = 100 * (duty/100);//rpm alterado para ficar mais visivel todo o processo
  return rpm;
}

float convert_rpm_rad_s(float rpm){
  float vel_rad_s = (rpm*2*PI)/60;//1 rot = 2PI rad e 1 min = 60s
  return vel_rad_s;
}

float get_velocidade_linear (float duty){//Essa é a velocidade que o  robo vai andar, considera igual pra as duas rodas. Vai depender da vel angular, q por sua vez depende da alimentacao do motor a ser ajustada pelo pwm
  float rpm = get_rpm(duty);
  float vel_rad_s = convert_rpm_rad_s(rpm);
  float vel_linear = vel_rad_s * RAIO_RODA; //v_linear = v_angular * raio
  return vel_linear;
}

float get_spin_vel(float duty){//Essa vai ser a velocidade de giro do robo. Necessario anexar o desenho pra entender a formula no relatorio
  float wheel_vel_linear = get_velocidade_linear(duty);
  float spin_vel = (2*wheel_vel_linear)/ (2*D_RODA_CENTROIDE);
  return spin_vel;
}

void move_robot_forward(float duty){//Passo o duty, pq movimento vai depender disso para variar a velocidade nos trechos. distancia - m
  set_pwm8bits_duty_cycle(duty);  
  set_motor1(true, FORWARD);//ligando o motor
  set_motor2(true, FORWARD);
}

/*por enquanto, ta uma funcao acessorio, no percursso o robo so vai pra frente e gira*/
void move_robot_backwards(float duty, float distancia){//distancia - m
  float vel_robo = get_velocidade_linear(duty); 
  float tempo_movimento = distancia/vel_robo;
  set_motor1(true, BACKWARDS);//ligando o motor
  set_motor2(true, BACKWARDS);
  _delay_ms(tempo_movimento*1000);
  set_motor1(false, BACKWARDS);//parando o motor
  set_motor1(false, BACKWARDS);
}

/*primeiro pego vel de giro do robo, que vai depender da rotacao, que vai depender do duty cycle do pwm para os didferentes percurssos*/
void spin_robot2(float duty, float angle){//angulo em grau
  float vel_giro = get_spin_vel(duty);
  float angle_rad = fabs(angle)*((2*PI)/360);//supondo q passa o angulo em graus, faz a conversao
  float tempo_giro = angle_rad/vel_giro; //vel_giro ta em rad/s, entao sai em segundo aqui
  if(angle >= 0){
  	set_motor1(true, FORWARD);
    set_motor2(true, BACKWARDS);
  }else{
  	set_motor1(true, BACKWARDS);
    set_motor2(true, FORWARD);
  }
  _delay_ms(tempo_giro*1000);//posso usar pq nao to fazendo nada enquanto gira
}
  
void spin_robot(float duty, int sentido_rotacao){
  set_pwm8bits_duty_cycle(duty);
  if(sentido_rotacao == 0){
  	set_motor1(true, FORWARD);
    set_motor2(true, BACKWARDS);	
  }else{
   	set_motor1(true, BACKWARDS);
    set_motor2(true, FORWARD);
  }
}

void print_medicoes()
{
    println("Medicoes");	
  	print("Ponto: ");
    print_int(trajeto-1);
    print("; temp: ");
  	print2(temp_string);
  	print("; gas_0: ");
  	print2(gas0_string);
  	print("; gas_90: ");
  	print2(gas90_string);
  	print("; gas_180: ");
  	print2(gas180_string);
  	print("; luz: ");
  	print2(luz_string);
  	println("");
}

void fill_temp_buffer(){
  String stringTemperatura = "";
  
  if(tempC >= -40 && tempC < 0){
    stringTemperatura += "Muito Frio";
  }else if(tempC >= 0 && tempC < 20){
  	stringTemperatura += "Frio";
  }else if (tempC >= 20 && tempC < 80){
  	stringTemperatura += "Quente";
  }else{
    stringTemperatura += "Muito Quente";
  }
  strcpy(temp_string, stringTemperatura.c_str());	
}

const char * get_gas_label(){
  if(n_gas_level >= 0 && n_gas_level < 35){
    return "Gas Desconsiderado";
  }else if(n_gas_level >= 35 && n_gas_level < 50){
  	return "Gas observado";
  }else if (n_gas_level >= 50 && n_gas_level < 75){
  	return "Muito gas";
  }else{
    return "Gas critico";
  }
}

void fill_gases(){
  String stringGas0 = "";
  String stringGas90 = "";
  String stringGas180 = "";
  
  n_gas_level = n_gas_0;
  stringGas0 += get_gas_label();
  
  n_gas_level = n_gas_90;
  stringGas90 += get_gas_label();
  
  n_gas_level = n_gas_180;
  stringGas180 += get_gas_label();
  
  strcpy(gas0_string, stringGas0.c_str());
  strcpy(gas90_string, stringGas90.c_str());
  strcpy(gas180_string, stringGas180.c_str());
}

void fill_luz(){
  String stringLuz = "";
  
  if(n_luz >= 0 && n_luz < 25){
    stringLuz += "Pouca Luz";
  }
  else if(n_luz >= 25 && n_luz < 50){
  	stringLuz += "Luz Observada";
  }
  else if (n_luz >= 50 && n_luz < 75){
  	stringLuz += "Muita luz";
  }
  else{
    stringLuz += "Luz critica";
  }
  strcpy(luz_string, stringLuz.c_str());
}

void read_sensors(){
  set_green_led(true);
  
  //sensor de temperatura
  ADMUX =  0b01000000; //AVCC como Ref. ADC0 como entrada ( primeiros bits 01)
  ADCSRA |= 0b01000000;//inicia a conversao ADSC vai pra 1   
  while( !(ADCSRA & 0b00010000));//trava aqui ate terminar leitura ADIF =1
  temperaturaADC = ADC;
  float step = 5.0/1023.0;
  float tempV = step * temperaturaADC;
  tempC = ((tempV-0.5)/10) * (1000);
  fill_temp_buffer();
  
   //sensor de gas
  /********** Leitura Gas INCIO *************/
  ADMUX =  0b01000001; //AVCC como Ref. ADC0 como entrada ( primeiros bits 01)
  
  servo_angle_0();
  _delay_ms(1000);
  ADCSRA |= 0b01000000;//inicia a conversao ADSC vai pra 1
  while( !(ADCSRA & 0b00010000));//trava aqui ate terminar leitura ADIF =1
  gasADC = ADC;
  n_gas_0 = (gasADC/1023)*100;
    
  //sensor de movimento - em 0 graus
  if((PIND & 0b10000000) == 128){
  	println("Movimento detectado em 0 graus");
  }
  
  servo_angle_90();
  _delay_ms(1000);
  ADCSRA |= 0b01000000;//inicia a conversao ADSC vai pra 1
  while( !(ADCSRA & 0b00010000));//trava aqui ate terminar leitura ADIF =1
  gasADC = ADC;
  n_gas_90 = (gasADC/1023)*100;
    
  //sensor de movimento - em 90 graus
  if((PIND & 0b10000000) == 128){
  	println("Movimento detectado em 90 graus");
  }
  
  servo_angle_180();
  _delay_ms(1000);
  ADCSRA |= 0b01000000;//inicia a conversao ADSC vai pra 1
  while( !(ADCSRA & 0b00010000));//trava aqui ate terminar leitura ADIF =1
  gasADC = ADC;
  n_gas_180 = (gasADC/1023)*100;
  
  fill_gases();
  
  //sensor de movimento - em 180 graus
  if((PIND & 0b10000000) == 128){
  	println("Movimento detectado em 180");
  }
  
  servo_angle_0();
  _delay_ms(1000);
  /********** Leitura Gas FIM *************/
  
  //sensor de luminosidade
  ADMUX =  0b01000010; //AVCC como Ref. ADC0 como entrada ( primeiros bits 01)
  ADCSRA |= 0b01000000;//inicia a conversao ADSC vai pra 1
  while( !(ADCSRA & 0b00010000));//trava aqui ate terminar leitura ADIF =1
  luzADC = ADC;
  n_luz = (luzADC/1023)*100;
 
  fill_luz();
  
  set_green_led(false);
  print_medicoes();
}

void piscar_leds(){
  if(toggle_leds == false){
  	set_red_led(true);
    set_green_led(false);
    toggle_leds = true;
  }else{
  	set_green_led(true);
    set_red_led(false);
    toggle_leds = false;
  }
}

void print(const char* message)
{
	int i =0;
  while(message[i] != '\0'){
  	while (!( UCSR0A & (0b00100000))); /* Wait for empty transmit buffer       */
	UDR0 = message[i];
    i++;
  }
}
 
void print2(char* message)
{
  int i =0;
  while(message[i] != '\0'){
  	while (!( UCSR0A & (0b00100000))); /* Wait for empty transmit buffer       */
	UDR0 = message[i];
    i++;
  }
}

void print_int(int valor)
{
  int i =0;
  String stringOne = String(valor);
  strcpy(buffer_int, stringOne.c_str());

    while(buffer_int[i] != '\0'){
  		while (!( UCSR0A & (0b00100000))); /* 1<<UDRE0 Wait for empty transmit buffer       */
		UDR0 = buffer_int[i];
    	i++;
  }
}

void print_float(float valor)
{
  int i =0;
  int valor_int_2decimals = round((valor* 100));
  float valor_float_2decimals = valor_int_2decimals;
  valor_float_2decimals = valor_float_2decimals/100;
  
  String stringOne = String(valor_float_2decimals);
  if (stringOne.length() == 0){
    valor_int_2decimals = valor_int_2decimals/100;
  	stringOne = String(valor_int_2decimals);
  }
  strcpy(buffer_float, stringOne.c_str());
  
  while(buffer_float[i] != '\0'){
  	while (!( UCSR0A & (0b00100000))); /* Wait for empty transmit buffer       */
	UDR0 = buffer_float[i];
    i++;
  }
}

void println(const char* message)
{
	int i =0;
  while(message[i] != '\0'){
  	while (!( UCSR0A & (0b00100000))); /* Wait for empty transmit buffer       */
	UDR0 = message[i];
    i++;
  }
  while (!( UCSR0A & (0b00100000)));   /* Wait for empty transmit buffer       */
  UDR0 = '\n';	
}

void print_status(){

  print("Total_time: ");
  print_float(total_time);
  print("; Trajeto: ");
  print_int(trajeto);
  print("; rx: ");
  print_float(robo.x);
  print("; ry: ");
  print_float(robo.y);
  print("; ra: ");
  print_float(robo.angle);
  print("; e: ");
  print_int(estado_robo);
  print("; duty: ");
  print_float(duty_atual);
  println("");
}

unsigned char recebeDado(){
  //bit rxc sinaliza que tem bytes nao lidos no buffer
  while(!(UCSR0A & (0b10000000)));
  return UDR0;
}

void setup()
{
  /*PC5 PC4 PC3 PC2 PC1 PC0 
  DDRC = 0 -> entrada / DDRC = 1 -> saida
  A0(PC0): conversor AD, sensor de temperatura (entrada).
  A1(PC1): conversor AD, sensor de gas (entrada).
  A2(PC2): conversor AD, sensor luminosidade (entrada).
  A3(PC3): 
  A4(PC4): 
  A5(PC5): liga/desliga robo (entrada).
  */
  DDRC &= 0b10000000;
  
  /*PD7 PD6 PD5 PD4 PD3 PD2 PD1 PD0 
  DDRD = 0 -> entrada / DDRD = 1 -> saida
  D0(PD0): 
  D1(PD1): 
  D2(PD2): saida para led vermelho
  D3(PD3): saida para led verde 
  D4(PD4): 
  D5(PD5): 
  D6(PD6): saida para pwm
  D7(PD7): entrada pra sensor de movimento - digital
  */
  
  DDRD |= 0b01001110;
  
  /*PB7 PB6 PB5 PB4 PB3 PB1 PB0
  13(PB5): inputs para motor 1 (saida).
  12(PB4): inputs para motor 1 (saida).
  11(PB3): PWM do servo motor
  10(PB2): inputs para motor 2 (saida).
  9(PB1) :
  8(PB0) : inputs para motor 2 (saida).
  */
  DDRB |= 0b00111101;
  
  bool robot_on = false;//inicializa robo desligado
  
  /*PWM motor DC*/
  init_pwm8bits();
  set_pwm8bits_duty_cycle(100);
  
  //PWM servor motor
  init_pwm8bits_servo();
  servo_angle_0();
  
  ADCSRA |= 0b01000000; //Inicializar o ADC 
  
  //contador 1 de 16 bits para interrupcao
  TCCR1A = 0b10000000;//modo normal de operacao(4 primeiros bits  da esq) TOV1 flag quando atinge maximo
  //o bit 3 habilita o CTC
  // (1/0.1) = 16000000/(1024*2*(1+X)
  // (x+1) = ( (16*10^6)*(0.1) ) / (1024*2)
  TCCR1B = 0b00001101;//(pre scaling de 1024 3 ultimos bits direita) (bit 4 bit da direita ativa o CTC, o valor de comparacao ta guardado em OCR1A)
  OCR1A = 15;//valor da contagem 0 -> 1560, ai ele dispara interrupcao e zera o contador
  //ts é o tempo que eu quero que cada interrupção seja chamada e ts= 1s.

  //Habilito o overflow no bit 1
  TIMSK1 = 0b00000010;
  
  sei();
  
  //para teste.
  // Serial.begin(9600);
  unsigned int ubrr = 3;//250k de taxa de transmissao
  
  /* Set Baudrate  */
  UBRR0H = 0; // Shift the 16bit value ubrr 8 times to the right and transfer the upper 8 bits to UBBR0H register.
  UBRR0L = ubrr;    // Copy the 16 bit value ubrr to the 8 bit UBBR0L register,Upper 8 bits are truncated while lower 8 bits are copied

  UCSR0C = 0b00000110;       /* Set frame format: 8data, 1stop bit  */
  UCSR0B = 0b10011000; /* Enable  transmitter  and receiver and interruption  (1<<RXCIE0) | (1<<TXEN0)| (1<<RXEN0)*/
  
  print_status();
  
  trajeto = 1;
  estado_robo = 1;
  
}
 

void loop()
{
// bool ligado = is_on() || switch_usart == 108 || switch_usart == 76;
  
  if(is_on()){
    //if(ligado == true){
      if(estado_robo == 0){
      shutdown_robot();
      duty_atual = 0;
    }
    if(estado_robo == 1){//alinhar o robo com o objetivo
    	set_red_led(true);
      	struct Ponto_coleta p_atual = pegar_ponto_trajeto(trajeto);
      	float spin_angle = calculate_angle(robo, p_atual );
      	float angle_dif = spin_angle - robo.angle;
      
      if(angle_dif >= 0){
        if (angle_dif > tol_ang_1 ){
        	duty_atual = 25;
        }
        else{
        	duty_atual = 5;
        }
        spin_robot(duty_atual, 1);
        motor_ligado = true;
        robo.sentido_giro = 1;
      }else{
        angle_dif = fabs(angle_dif);
        if (angle_dif > tol_ang_1 ){
        	duty_atual = 25;
        }
        else{
        	duty_atual = 5;
        }
        spin_robot(duty_atual, 2);
        motor_ligado = true;
        robo.sentido_giro = 2;      	
      }
      
      if(fabs(angle_dif) < tol_ang_2){
      	stop_motors();
        duty_atual =0;
        motor_ligado = false;
        robo.sentido_giro = 0;
        estado_robo = 2;//robo ja esta alinhado, agora vai pro mover      
      	set_red_led(false);
      	print_status();
      }
    }
    
    else if(estado_robo == 2){
      if(trajeto == 1){
      	set_red_led(true);
        estado_robo++;
      }
      else if(trajeto >= 2 || trajeto <= 5){
      	set_red_led(true);
        read_sensors();
        estado_robo++;
        set_red_led(false);
      }
      else{
      	estado_robo++;
      }
      print_status();
      
    }
    
    else if(estado_robo == 3){//robo entra em movimento
    	set_red_led(true);
      	struct Ponto_coleta p_atual = pegar_ponto_trajeto(trajeto);
      	float distancia_ate_objetivo = calculate_distance(robo, p_atual);
      	
        if (distancia_ate_objetivo > tol_dist_1){
          duty_atual = pegar_duty_trajeto(trajeto);
          move_robot_forward(duty_atual);
          robo.sentido_robo = 1;
          motor_ligado = true;
      	}
      	else if(distancia_ate_objetivo > tol_dist_2){
          duty_atual = 10;
          move_robot_forward(duty_atual);
          robo.sentido_robo = 1;
          motor_ligado = true;      	
      	}
        else{
          trajeto++;
          print_status();
          stop_motors();
          motor_ligado = false;
          robo.sentido_robo = 0;
          if(trajeto >= 6){
              estado_robo = 4;
              set_red_led(false);
          }else{
              estado_robo = 1;
              set_red_led(false);
          }
      	}	
    }
    
    else if(estado_robo == 4){
    	set_red_led(true);
      	read_sensors();
      	estado_robo = 5;
      	rotacao_acumulada = 0;
      	set_red_led(false);
        print_status();
    }
    
    else if(estado_robo == 5){
      	duty_atual = 25;
    	spin_robot(duty_atual, 1);
      	motor_ligado = true;
     	robo.sentido_giro = 1;
      	if(count_led >= 1000){
          piscar_leds();
          count_led = 0;
      	}
      	if(rotacao_acumulada > 720){
      		stop_motors();
            set_red_led(false);
          	set_green_led(false);
        	motor_ligado = false;
        	estado_robo = 0;
          	duty_atual = 0;
          	robo.sentido_giro = 0;
          	print_status();
          	exit(0);
      	}
    } 
    
    // Imprimi o status
    if(count_print >= 500){
        count_print = 0;
    	print_status();
    }
 
      if(is_on() == false){
      	if(switch_usart != 108 && switch_usart != 76){
       	stop_motors();
        set_red_led(false);
        set_green_led(false);
        motor_ligado = false;
        estado_robo = 0;
        duty_atual = 0;
        print_status();
        exit(0);
      	}
      }
      
  }
}

ISR(TIMER1_COMPA_vect){ 
  if (is_on()){
      timer_interruption_flag = true;
      total_time = total_time + interruption_interval;
      count_print++;
      count_total++;
      count_led++;

    if(motor_ligado == true){
      //atualizar a orientacao do robo quando robo ta girando
      if(robo.sentido_giro == 1){
          vel_giro = get_spin_vel(duty_atual);
          delta_angle = vel_giro *interruption_interval;
          delta_angle_graus = (delta_angle*180)/PI;
          robo.angle += delta_angle_graus;
          //Serial.print(delta_angle_graus);
          rotacao_acumulada += delta_angle_graus;
      }
      if(robo.sentido_giro == 2){
          vel_giro = get_spin_vel(duty_atual);
          delta_angle = -vel_giro *interruption_interval;
          delta_angle_graus = (delta_angle*180)/PI;
          robo.angle += delta_angle_graus;
          rotacao_acumulada += delta_angle_graus;
      }
      if(robo.angle > 180){
        robo.angle -= 360;
      }
      if(robo.angle < -180){
        robo.angle += 360;
      }

      //atualizando posicao x e y quando robo ta se movimentando
      if(robo.sentido_robo == 1){
          vel_linear_robo = get_velocidade_linear(duty_atual);
          angle_robo_rad = (robo.angle* PI)/180;
          delta_vel = vel_linear_robo * interruption_interval;
          robo.x += delta_vel * cos(angle_robo_rad);
          robo.y += delta_vel * sin(angle_robo_rad);
      }
    }
  }
}

ISR(USART_RX_vect){
  //receber os comandos pra desligar ou ligar
  switch_usart = recebeDado();
  if(switch_usart == 108 || switch_usart == 76){
  	PINC |= 0b00100000;
  }else if (switch_usart == 100 || switch_usart == 68){
  	PINC &= 0b11011111;
  }
  else{
    if(is_on()){
    	PINC |= 0b00100000;
    }else{
    	PINC &= 0b11011111;
    }
  }
}