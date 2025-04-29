#include <Arduino.h>

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

void plot(int x, int y, int h, int w);
void menu9();

void setup() {

}

void loop() {
  // put your main code here, to run repeatedly:
}

void menu9(){
  int margin=10;
  int x=80;
  for(int ii=0;ii<9;ii++){
    plot((ii/3*x)+margin,(ii%3*x)+margin,x-2*margin,x-2*margin);
  }
  for(int ii=0;ii<3;ii++){
    plot((ii*x)+margin,(3*x)+margin,x-2*margin,x-2*margin);
  }
}

void plot(int x, int y, int h, int w){
//NO OPerator
}