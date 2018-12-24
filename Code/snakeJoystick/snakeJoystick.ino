/**

   Author: Mike Fu
   Final Project for Embedded Systems
   Colby College

   This is the code for the tetris project
*/

#include <avr/io.h>
#include <Wire.h>
#include "USART.h"
#include "Adafruit_LEDBackpack.h"

/* Global variables */
int width = 8;
int height = 16;

// Game Logic
int x, y, fruitX, fruitY, score;
bool gameOver = false;
int delay_time = 500;

int tailX[128], tailY[128];
int nTail;

enum Direction {STOP = 0, LEFT, RIGHT, UP, DOWN};
Direction dir;
Direction buttonDir;

int ms = 0;
int ms2 = 0;
bool senseX = true;

uint16_t sensedADC;     // most recent ADC measurement
float meanX = 0;         // the online mean
float meanY = 0;
/* this array will contain the field.
    Field size is 16 rows by 8 columns
    indexed by pi = row_num * field_width + col_num
*/
bool field[16][8];

bool useButton = false;

/*
   Pin Change Interrupt 2

   Pad 2 - Left
   Pad 3 - Down
   Pad 4 - Right
   Pad 5 - Up
*/
ISR(PCINT2_vect) {

  // PD2
  if (PIND &= 0b00000100) {
    buttonDir = LEFT;
  }
  // PD3
  if (PIND &= 0b00001000) {
    buttonDir = DOWN;
  }
  // PD4
  if (PIND &= 0b00010000) {
    buttonDir = RIGHT;
  }
  // PD5
  if (PIND &= 0b00100000) {
    buttonDir = UP;
  }

  useButton = true;
}


Adafruit_8x16minimatrix matrix = Adafruit_8x16minimatrix();

/*
   Controls the ADC
   ADC updates every 1ms
   alternates b/w X and Y every 100 ms
*/
ISR(TIMER0_COMPA_vect) {
  ms++;
  int switchTime = 100;

  if (senseX) {
    // sets ADC input to PC0
    ADMUX &= 0b11111000;
  }
  else {
    // sets ADC input to PC1
    ADMUX &= 0b11111000;
    ADMUX |= 0b00000001;
  }

  // switches between X and Y sensation every "switchTime" ms
  if (ms % (2 * switchTime) == 0) {
    senseX = true;
  }
  else if (ms % (2 * switchTime) == switchTime) {
    senseX = false;
  }

  // resets ms to avoid overflow
  if (ms == 10000) {
    ms = 0;
  }
}

/*
   Controls the Game Logic
   alternates b/w X and Y every 100 ms
*/
ISR(TIMER2_COMPA_vect) {
  ms2++;

//  if (senseX) {
//    // sets ADC input to PC0
//    ADMUX &= 0b11111000;
//  }
//  else {
//    // sets ADC input to PC1
//    ADMUX &= 0b11111000;
//    ADMUX |= 0b00000001;
//  }

  // resets ms to avoid overflow
  if (ms2 == 10000) {
    ms2 = 0;
  }
}


ISR(ADC_vect) {
  int window = 10;   // small window because the joy stick is bloody accurate

  sensedADC = ADCL;                 // must read low byte first
  sensedADC |= (ADCH & 0x03) << 8;  // 10-bit precision

  if (senseX) {
    meanX = float(meanX * (window - 1) + sensedADC) / float(window);
  }
  else {
    meanY = float(meanY * (window - 1) + sensedADC) / float(window);
  }
}

void draw() {

  // draws snake
  matrix.clear();
  matrix.drawPixel(x, y, LED_ON);
  for (int i = 0; i < nTail; i++) {
    matrix.drawPixel(tailX[i], tailY[i], LED_ON);
  }

  // draws fruit
  matrix.drawPixel(fruitX, fruitY, LED_ON);
  matrix.writeDisplay();

}

void gameLogic() {

  int prevX = tailX[0];
  int prevY = tailY[0];
  int prev2X, prev2Y;
  tailX[0] = x;
  tailY[0] = y;

  for (int i = 1; i < nTail; i++) {
    prev2X = tailX[i];
    prev2Y = tailY[i];
    tailX[i] = prevX;
    tailY[i] = prevY;
    prevX = prev2X;
    prevY = prev2Y;
  }

  switch (dir) {
    case LEFT:
      y++;
      break;
    case DOWN:
      x++;
      break;
    case RIGHT:
      y--;
      break;
    case UP:
      x--;
      break;
  }
}

void checkGameStatus() {

  // checks if the snake hit a bound
  if (x < 0 || x > width || y < 0 || y > height) {
    gameOver = true;
  }

  // checks if the snake has hit itself
  for (int i = 0; i < nTail; i++) {
    if (x == tailX[i] && y == tailY[i]) {
      gameOver = true;
      break;
    }
  }

  // checks if fruit has been eaten
  if (x == fruitX && y == fruitY) {
    score += 10;
    nTail++;
    fruitX = rand() % width;
    fruitY = rand() % height;
    delay_time = max(delay_time - 30, 100);
  }
}

/** reads the ADC from the analog joystick and returns whether

    Return 0 for up
    return 1 for right
    return 2 for down
    return 3 for left
    return 4 for middle
*/
Direction decipherJoystick() {

  // takes care of bottom right and bottom left corner

  // rightish direction
  if (meanX > 750) {
    if (abs(meanY - 420) > abs(meanX - 750) && meanY < 420) {
      useButton = false;
      return DOWN;     // more down than right
    }
    else if (abs(meanY - 750) > abs(meanX - 750) && meanY > 750) {
      useButton = false;
      return UP;     // more up than right
    }
    // just right
    useButton = false;
    return RIGHT;
  }
  // leftish direction
  else if (meanX < 420) {
    if (abs(meanY - 420) > abs(meanX - 420) && meanY < 420) {
      useButton = false;
      return DOWN;     // more down than left
    }
    else if (abs(meanY - 750) > abs(meanX - 420) && meanY > 750) {
      useButton = false;
      return UP;     // more up than left
    }
    useButton = false;
    return LEFT;
  }
  // down
  else if (meanY < 420) {
    useButton = false;
    return DOWN;
  }
  // up
  else if (meanY > 750) {
    useButton = false;
    return UP;
  }
  // in middle
  else {
    if (useButton)
      return buttonDir;
  }

}

/*
    Custom delay function for variable delays
*/
void delayX(int delay_time) {
  while (delay_time--) {
    _delay_ms(1);
  }
}

int main() {

  initUSART();
  init();
  printString("");

  matrix.begin(0x70);  // pass in the address

  /* Initialize GPIO */


  /* Initializes ADC */
  ADMUX = 0b01000000;   // read from PC0
  ADCSRA = 0b11101100;  // enable ADC
  ADCSRB = 0b00000011;  // on TIMER0A Interrupt

  /* Initialize Timers */

  // enables TIMER0_COMPA
  // 1ms per interrupt
  TCCR0A = 0b00000010;
  TCCR0B = 0b00000011;    // scale by 64
  OCR0A = 250;
  TIMSK0 |= 0b00000010;

  // Enable TIMER2
  TCCR2B = 0b00000100;    // scale by 64
  OCR2A = 250;
  TIMSK2 |= 0b00000010;

  /* Configure Interrupts */
  // Configure Pin interrupt 2 (pad D)
  PCICR = 0b00000100;
  PCMSK2 = 0b00111100;

  SREG |= 0x80;

  /* Game controls */
  dir = STOP;
  buttonDir = dir;
  x = width / 2;
  y = height / 2;
  fruitX = rand() % width;
  fruitY = rand() % height;
  score = 0;

  while (!gameOver) {
    //    printWord(decipherJoystick());
    //    printString("\n");
    //    move(decipherJoystick());
    dir = decipherJoystick();
    gameLogic();
    draw();
    checkGameStatus();
    // plan to use timers instead
    delayX(delay_time);
  }

  // displays post game stats
  matrix.setTextSize(1);
  while (true) {
    matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
    matrix.setTextColor(LED_ON);
    matrix.setRotation(1);
    for (int8_t x = 7; x >= -60; x--) {
      matrix.clear();
      matrix.setCursor(x, 0);

      char s[200];
      sprintf(s, "SCORE: %d", score);
      matrix.print(s);
      matrix.writeDisplay();
      delay(100);
    }
  }
  matrix.setRotation(0);
}
