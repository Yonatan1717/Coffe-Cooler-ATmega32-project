#define F_CPU 1000000UL
#define __DELAY_BACKWARD_COMPATIBLE__
#include <avr/io.h>
#include <util/delay.h>
#include <avrLib.h>



int main(){
  // Sett kontrollpinner (RS = PC0, EN = PC2)
  LCD_cmd_OM;
  LCD_data_OM;

  _delay_ms(20);



  LCD_ins(0x38); 
  LCD_ins(0x0F);
  LCD_ins(0x06); 
  LCD_ins(0x01); 
 

 
  char *hello = "hello";
  while(*hello)
  {
    LCD_data(*hello++);
  }
  
  LCD_data(0xFE);

  char *world = "world";
  while (*world)
  {
    LCD_data(*world++);
  }

  LCD_ins((1<<PD7)| (0x40));
  char *worlddd = "how are you";
  while (*worlddd)
  {
    LCD_data(*worlddd++);
  }

  while (1);
}
