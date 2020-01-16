#include <stdio.h>
#include <math.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

// U
#define GPIO_PWM0A_OUT 13   //Set GPIO 19 as PWM0A 
#define GPIO_PWM0B_OUT 12   //Set GPIO 18 as PWM0B

//
//#define GPIO_PWM1A_OUT 17   //Set GPIO 17 TX2 as PWM1A 
//#define GPIO_PWM1B_OUT 16   //Set GPIO 16 RX2 as PWM1B

#define GPIO_PWM1A_OUT 14   //Set GPIO 22 as PWM1A 
#define GPIO_PWM1B_OUT 27   //Set GPIO 23 as PWM1B
//
#define GPIO_PWM2A_OUT 26   //Set GPIO 15 as PWM2A
#define GPIO_PWM2B_OUT 25   //Set GPIO 14 as PWM2B



int Nova = 0;

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

#define     TAMANHOTABELA          1500


typedef struct _Motor {
    uint8_t     Status;
    uint8_t     Tipo;
    uint8_t     Sentido;
    uint32_t    Ciclos;
    int         W,V,U;
    int         PWMHZ;
    int         DUTYMAX;

    struct      TB {
        uint16_t Sample[TAMANHOTABELA];
        int      NumeroSamples;
        int      Frequencia;  
    } Tabela[2];

    int TabelaAtual;

} Motor_t;

Motor_t Motor;
#define     HZPARAPULSOS( M, H )      ( M.PWMHZ / H)

void NovaTabela(Motor_t *M, int frequencia) {    
    
    float D = 1.0f;
    int Proxima;
    if (Motor.TabelaAtual == 0) Proxima = 1; else Proxima = 0;     
    
    configASSERT(frequencia > 0 && frequencia < 200 && M != NULL && M->Tabela[Proxima].NumeroSamples == 0)

    M->Tabela[Proxima].Frequencia = frequencia;

    if (frequencia < 60) D = (float) 60.0f / M->Tabela[Proxima].Frequencia; // else D = 1.0f;

    int Limite  = M->PWMHZ / M->Tabela[Proxima].Frequencia;

    float R = 6.28318530718f / Limite, r;
    int X = 1 + M->DUTYMAX / 2;
    int XX = (X * 1.15465465465465465465f) / D; 
    //printf ("%d %d %f\n", X, XX, R);
    for (int i = 0; i < Limite; i++) {
        r = i * R;
        M->Tabela[Proxima].Sample[i] = X + XX * ( sin( r ) + sin( 3.0f * r ) / 6.0f );
        //printf ("%d ", M->Tabela[Proxima].Sample[i]);
    }
    M->Tabela[Proxima].NumeroSamples = Limite;
}

static void IRAM_ATTR isr_handler()
{
    uint32_t mcpwm_intr_status;
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
    int Limite =  Motor.Tabela[Motor.TabelaAtual].NumeroSamples;
    int Proxima;

    if (Motor.TabelaAtual == 0) Proxima = 1; else Proxima = 0; 

    if (Limite > 0) {
        if (mcpwm_intr_status & BIT(3)) { 
            Motor.U += 1; 
            if ( Motor.U >= Limite )  { 
                Motor.U = 0;
                Motor.Ciclos++;
                if (Motor.Tabela[Proxima].NumeroSamples > 0) {
                    Motor.Tabela[Motor.TabelaAtual].NumeroSamples = 0;
                    Motor.TabelaAtual = Proxima;
                    Limite =  Motor.Tabela[Motor.TabelaAtual].NumeroSamples;    
                    Motor.W = Motor.Tabela[Motor.TabelaAtual].NumeroSamples / 3;
                    Motor.V = Motor.W + Motor.W;                
                    Motor.U = 1;
                }
                
            
            }
            int D = Motor.Tabela[Motor.TabelaAtual].Sample[Motor.U];
            //portENTER_CRITICAL(&mcpwm_spinlock);
                MCPWM[0]->channel[0].cmpr_value[0].cmpr_val = D;
                MCPWM[0]->channel[0].cmpr_value[1].cmpr_val = D;
                MCPWM[0]->channel[0].cmpr_cfg.a_upmethod = BIT(0);
                MCPWM[0]->channel[0].cmpr_cfg.b_upmethod = BIT(0);
            //portEXIT_CRITICAL(&mcpwm_spinlock);
            //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, D);
            //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, D);           
        //}
        //if (mcpwm_intr_status & BIT(4)) { 
            Motor.W += 1; 
            if ( Motor.W >= Limite ) Motor.W = 0;
            D = Motor.Tabela[Motor.TabelaAtual].Sample[Motor.W];
                MCPWM[0]->channel[1].cmpr_value[0].cmpr_val = D;
                MCPWM[0]->channel[1].cmpr_value[1].cmpr_val = D;
                MCPWM[0]->channel[1].cmpr_cfg.a_upmethod = BIT(0);
                MCPWM[0]->channel[1].cmpr_cfg.b_upmethod = BIT(0);
            //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, D);
            //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, D);          
        //}
        //if (mcpwm_intr_status & BIT(5)) { 
            Motor.V += 1; 
            if ( Motor.V >= Limite ) Motor.V = 0;
            D = Motor.Tabela[Motor.TabelaAtual].Sample[Motor.V];
                MCPWM[0]->channel[2].cmpr_value[0].cmpr_val = D;
                MCPWM[0]->channel[2].cmpr_value[1].cmpr_val = D;
                MCPWM[0]->channel[2].cmpr_cfg.a_upmethod = BIT(0);
                MCPWM[0]->channel[2].cmpr_cfg.b_upmethod = BIT(0);
            //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, D);
            //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, D);  
        }
    }

    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
}


// mcpwm_set_duty_in_us()


static void mcpwm_example_config(void *arg)
{
   
    mcpwm_config_t pwm_config;

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_PWM2B_OUT);

	
	
	#warning CHANGE the clock prescale in mcpwm.c file to make possible get 30000hz of pwm frequency (15Khz in center aligned mode)
	
    pwm_config.frequency = 30000;    //frequency 
    pwm_config.cmpr_a = 95.0;       // Duty em porcentagem
    pwm_config.cmpr_b = pwm_config.cmpr_a;       
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   //Configure PWM1A & PWM1B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);   //Configure PWM2A & PWM2B with above settings       

    Motor.PWMHZ = mcpwm_get_frequency(MCPWM_UNIT_0,MCPWM_TIMER_0) / 2;
    Motor.DUTYMAX = MCPWM[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].period.period;
    Motor.Ciclos = 0;
    Motor.TabelaAtual = 0;

    Motor.Tabela[0].NumeroSamples = 0;
    Motor.Tabela[0].Frequencia = 0;

    Motor.Tabela[1].NumeroSamples = 0;
    Motor.Tabela[1].Frequencia = 0;

    NovaTabela(&Motor, 30);
    
    Motor.TabelaAtual = 1;
    
    Motor.U = 0;
    Motor.W = Motor.Tabela[Motor.TabelaAtual].NumeroSamples / 3;
    Motor.V = Motor.W + Motor.W;


    printf ("Motor Frequencia/Frequency: %d\n", Motor.Tabela[Motor.TabelaAtual].Frequencia);
    printf ("Motor PWMHZ: %d\n", Motor.PWMHZ);
    printf ("Motor W: %d\n", Motor.W);
    printf ("Motor V: %d\n", Motor.V);
    printf ("Motor U: %d\n", Motor.U);
    printf ("Motor TabelaLimite: %d\n", Motor.Tabela[Motor.TabelaAtual].NumeroSamples);

    

	// Deadtime 
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 100, 100); // 80 = 1us   
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 100, 100);   
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 100, 100);  

    MCPWM[MCPWM_UNIT_0]->int_ena.val = BIT(3) /*| BIT(4) | BIT(5)*/; // /*A PWM timer X TEZ event will trigger this interrupt*/  
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Motor.Tabela[Motor.TabelaAtual].Sample[Motor.U]);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, Motor.Tabela[Motor.TabelaAtual].Sample[Motor.U]); 

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, Motor.Tabela[Motor.TabelaAtual].Sample[Motor.W]);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, Motor.Tabela[Motor.TabelaAtual].Sample[Motor.W]);

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, Motor.Tabela[Motor.TabelaAtual].Sample[Motor.V]);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, Motor.Tabela[Motor.TabelaAtual].Sample[Motor.V]);

    //mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, 0); 
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, 1, 0);
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, 1, 0);  

    MCPWM[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].sync.out_sel = 1;

    int freq = 30,i = 1;
    
    for (;;) { 
                
        if (freq >= 150) i = -1;
        if (freq <= 11)  i = +1; 
        freq = freq + i;

        NovaTabela(&Motor, freq);
            printf ("Fequencia/Frequency: %d\n", Motor.Tabela[Motor.TabelaAtual].Frequencia);
            printf ("U %d ,V %d ,W %d\n", Motor.U , Motor.V , Motor.W);
            printf ("Tamanho Tabela/Size of Tabela: %d\n", Motor.Tabela[Motor.TabelaAtual].NumeroSamples);
            printf ("Ciclos/Cicles: %d\n", Motor.Ciclos);
        vTaskDelay ( 1000 / portTICK_PERIOD_MS); 
        
        
    }

    vTaskDelay ( 500 / portTICK_PERIOD_MS);
    vTaskDelete( NULL );

}

void Motor_Init(void)
{
    xTaskCreate(mcpwm_example_config, "Motor_Loop", 4096, NULL, 5, NULL);
}


