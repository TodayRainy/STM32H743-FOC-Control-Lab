#ifndef _BSP_H
#define _BSP_H

#include "sys.h"
#include "delay.h"
#include "my_usart.h"

#include "icm42688/icm42688p.h"



void BSP_Init(void);                
void BSP_RAM_Init(void);            
void BSP_Flash_Init(void);          
void BSP_Flash_mount(void);         


void BSP_Display_Init(void);        
void BSP_Touch_Init(void);          
#endif  

