#include <Arduino.h>
#include <fix_fft.h>  // https://github.com/kosme/fix_fft
#include <CyberLib.h> // из нее берется только UART_SendArray и UART_SendByte можно заменить на Serial.print()


#define UART_SPEED 1000000
// frequency limits
const uint16_t freqLL = 400;
const uint16_t freqLH = 800;
const uint16_t freqHL = 1800;
const uint16_t freqHH = 2300;

const uint8_t blinker = 13;                            // sqelch output chanell
#define ADC_CHANEL 0                             // audio input on A0
// const uint16_t samplingFrequency = 6250;               // sampling frequency
#define SAMPL_FREQ 6250
// const uint16_t ticks = F_CPU / 64 / samplingFrequency; // tick of timer for ADC interrupt
#define ticks F_CPU/64/SAMPL_FREQ
const int FFTe = 6;                                    // FFT size in bit
const uint8_t sqlHysto = 100;                          // time latency to close squelch

// bin of FFT  with our frequency
const uint16_t samples = (1 << FFTe); // FFT size
const uint8_t binLL = (freqLL * samples / SAMPL_FREQ) + 1;
const uint8_t binLH = (freqLH * samples / SAMPL_FREQ) + 1;
const uint8_t binHL = (freqHL * samples / SAMPL_FREQ) + 1;
const uint8_t binHH = (freqHH * samples / SAMPL_FREQ) + 1;

// const uint8_t binLL=7;
// const uint8_t binLH=7;
// const uint8_t binHL=17;
// const uint8_t binHH=39;

uint16_t powerLS[8];
uint16_t powerHS[8];
uint16_t powerLSS;
uint16_t powerHSS;

// uint16_t powerL;
// uint16_t powerH;

// adc flag
volatile bool adcFlag;

int16_t sqlStatus = 0;
uint16_t count = 0;
char p[8];    // itoa() conversion buffer
uint8_t q[8]; // itoa() conversion buffer

// buffer for ADC
char buffer1[samples];
char buffer2[samples];
char *adcBuffer = buffer1; // work buffer for ADC
char *vReal = buffer2;     // work buffer for FFT
char vImag[samples];

void initialize_ADC() // !!!МОЖНО ПЕРЕВЕСТИ НА ИСПОЛЬЗОВАНИЕ TIMER0!!!
{
  cli();
  // enable adc, auto trigger, interrupt enable, prescale=128
  ADCSRA = ((1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));

  // Timer/Counter 0 Compare Match A
  // ADCSRB = (( 1 << ADTS1 ) | ( 1 << ADTS0 ));

  // Timer/Counter 1 Compare Match B
  ADCSRB = ((1 << ADTS2) | (1 << ADTS0));

  // ref=AVcc + adc chan
  ADMUX = ((1 << REFS1) + (1 << REFS0) + ADC_CHANEL);
  sei();
}

void TimerOneInit()
{

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TIMSK1 = 0;

  TCCR1B = (1 << WGM12); // Configure for CTC mode 4 OCR1A=TOP

  OCR1B = ticks; // compare value
  OCR1A = ticks; // Set CTC TOP value, must be >= OCR1B

  // start timer, give it a clock
  // TCCR1B |= ( 1 << CS11 ) ;                    // Fcpu/8,    0.5us tick @ 16 MHz
  TCCR1B |= ((1 << CS11) | (1 << CS10)); // Fcpu/64,   4us   tick @ 16 MHz
  // TCCR1B |= ( 1 << CS12 ) ;                    // Fcpu/256,  16us  tick @ 16 MHz
  // TCCR1B |= (( 1 << CS10 ) | ( 1 << CS12 )) ;  // Fcpu/1024, 64us  tick @ 16 MHz

} // TimerOneInit

ISR(ADC_vect) // ADC conversion complete
{
  //  ADCresult = ADC;
  adcBuffer[count] = (ADC >> 2) - 127; // Read the ADC 0...1023 -> -127...127

  // считаем семплы в буффере
  count++;
  if (count == samples) // как буфер заполнился свапаем указатели устанавливаем флаг
  {
    count = 0;
    char *temp = adcBuffer;
    adcBuffer = vReal;
    vReal = temp;
    adcFlag = true;
  }

  TIFR1 = (1 << OCF1B); // clear Compare Match B Flag
}

void UART_SendString(char *buffer){
  while(*buffer)
  {
    UART_SendByte(*buffer);
    buffer++;
  }
}

void setup()
{
  D13_Out;
  UART_Init(UART_SPEED);
  TimerOneInit();
  initialize_ADC();
}

void loop()
{
label:

  if (adcFlag)
  {
    memset(vImag, 0, samples);
    fix_fft(vReal, vImag, FFTe, 0);

    uint16_t powerL = 0;
    uint16_t powerH = 0;
    // печатаем FFT
    for (uint8_t i = binLL; i <= binLH; i++)
    {
      powerL += (uint16_t)vReal[i] * (uint16_t)vReal[i] + (uint16_t)vImag[i] * (uint16_t)vImag[i];
    }
    for (uint8_t i = binHL; i <= binHH; i++)
    {
      powerH += (uint16_t)vReal[i] * (uint16_t)vReal[i] + (uint16_t)vImag[i] * (uint16_t)vImag[i];
    }
    powerL++;
    powerH++;
    powerLS[8] = powerLS[7];
    powerLS[7] = powerLS[6];
    powerLS[6] = powerLS[5];
    powerLS[5] = powerLS[4];
    powerLS[4] = powerLS[3];
    powerLS[3] = powerLS[2];
    powerLS[2] = powerLS[1];
    powerLS[1] = powerLS[0];
    powerLS[0] = powerL;

    powerHS[8] = powerHS[7];
    powerHS[7] = powerHS[6];
    powerHS[6] = powerHS[5];
    powerHS[5] = powerHS[4];
    powerHS[4] = powerHS[3];
    powerHS[3] = powerHS[2];
    powerHS[2] = powerHS[1];
    powerHS[1] = powerHS[0];
    powerHS[0] = powerH;

    powerLSS = ((powerLS[0] + powerLS[1] + powerLS[2] + powerLS[3])) + ((powerLS[4] + powerLS[5] + powerLS[6] + powerLS[7]));
    powerHSS = ((powerHS[0] + powerHS[1] + powerHS[2] + powerHS[3])) + ((powerHS[4] + powerHS[5] + powerHS[6] + powerHS[7]));

    if ((powerLSS * 3) > (powerHSS * 5)) // порог на включение 1.5
    {
      sqlStatus += (sqlHysto / 3) + 1;
      if (sqlStatus > sqlHysto)
        sqlStatus = sqlHysto;
    }
    else if ((powerLSS * 5) < (powerHSS * 6)) // порог на отключение 1.16
      sqlStatus--;

    if (sqlStatus < 0)
      sqlStatus = 0;

    UART_SendString(itoa(powerLSS, p, 10));
    UART_SendByte(' ');
    UART_SendString(itoa(powerHSS, p, 10));
    UART_SendByte(' ');
    UART_SendString(itoa(sqlStatus, p, 10));
    UART_SendByte('\n');

    if (sqlStatus == sqlHysto)
      PORTB |= (1 << PB5);
    else if (sqlStatus == 0)
      PORTB &= ~(1 << PB5);

    adcFlag = false;
  }

  goto label;
}
