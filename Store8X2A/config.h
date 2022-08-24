#ifndef _CONFIG_H_
#define _CONFIG_H_


#define RAMP_UINCPOW 7
#define RAMP_MAXERROR 1
#define RAMP_TO_POS_POW 4

//#define MOTC_PWM 1
//#define MOTD_PWM 2

#define MOTC_END K7
#define MOTC_ENDLEVEL 0
#define MOTC_A K1
#define MOTC_B K2

#define MOTD_END K8
#define MOTD_ENDLEVEL 0
#define MOTD_A K3
#define MOTD_B K4

//#define ANALOG_FILTER 5

//#define DMX_SLAVE_UART_NUM 	AUXSERIAL_NUM // 1 for 8X2A
//#define DMX_SLAVE_UART_PIN	AUXSERIAL_RX  // K6 for 8X2A
#define DMX_SLAVE_NBCHAN		511

#define MOTC_ENDLED K5
#define MOTD_ENDLED K9

#define DMX_ENABLE MA2

#endif // _CONFIG_H_

