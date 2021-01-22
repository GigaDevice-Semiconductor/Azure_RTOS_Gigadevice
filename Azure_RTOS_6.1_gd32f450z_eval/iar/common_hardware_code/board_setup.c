#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "gd32f4xx.h"
#include "gd32f450z_eval.h"

int  board_setup(void)
{

    /* configure 4 bits pre-emption priority */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    gd_eval_com_init(EVAL_COM0);
    gd_eval_led_init(LED1);
    gd_eval_led_off(LED1);
    gd_eval_led_init(LED2);
    gd_eval_led_off(LED2);
    gd_eval_led_init(LED3);
    gd_eval_led_off(LED3);

    return 0;
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    return ch;
}
