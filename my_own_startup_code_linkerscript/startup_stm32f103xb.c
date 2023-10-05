/*
*  @file   startup_stm32f103xb.c
*  @author Ahmed Radwan Ibrahim
*  @brief  startup code for stm32f103c8 Device
*                This module performs:
*                - Set the initial SP
*                - Set the initial PC == Reset_Handler,
*                - Set the vector table entries with the exceptions ISR address
*                  calls main(). 
*  @ref  startup_stm32f103xb.s file   
*/

#define NULL ((void*)0) 
typedef unsigned int uint32_t;
/*used to detremine the value of stack ptr*/
extern uint32_t _estack;      
uint32_t  * const MSP= (uint32_t*)&_estack;   /*1-why we use (&) 2- we use const because we will but it in the flash*/

/* start address for the initialization values of the .data section.defined in linker script */
extern uint32_t _sidata;
/* start address for the .data section. defined in linker script */
extern uint32_t _sdata;
/* end address for the .data section. defined in linker script */
extern uint32_t _edata;
/* start address for the .bss section. defined in linker script */
extern uint32_t _sbss;
/* end address for the .bss section. defined in linker script */
extern uint32_t _ebss;

/*extern the application's entry function */
extern int main(void);


/* Reset handeler implemntation */
void Reset_Handler(void){
    /*intialize the needed variables */
    uint32_t  Section_Size=0;
    uint32_t* Source_adrress=NULL; 
    uint32_t* Dest_adrress=NULL; 

    /* Call the clock system initialization function.*/
             /*to be done*/
    /* Copy the data segment initializers from flash to SRAM */
    Section_Size= &_edata - &_sdata ;
    Source_adrress= &_sidata ;
    Dest_adrress= &_sdata;
    for(uint32_t counter=0;counter<Section_Size;counter++)
    {
        *Dest_adrress++ = *Source_adrress++;
    }

    /* Zero fill the bss segment. */
    Section_Size= &_ebss - &_sbss ;
    Dest_adrress= &_sbss ;
    for(uint32_t counter=0;counter<Section_Size;counter++)
    {
        *Dest_adrress++=0;
    }

    /* Call the application's entry point.*/
    main();
}

/*default interrupt handeller for unimplemented interrupt*/
void Default_Handler(void) {
    while(1);
}


/************************* Weak definitions of interupt handlers *************************/
     void __attribute__((weak)) NMI_Handler                (void)	 { Default_Handler(); }
     void __attribute__((weak)) HardFault_Handler          (void)	 { Default_Handler(); }
     void __attribute__((weak)) MemManage_Handler          (void)	 { Default_Handler(); }
     void __attribute__((weak)) BusFault_Handler           (void)	 { Default_Handler(); }
     void __attribute__((weak)) UsageFault_Handler         (void)	 { Default_Handler(); }
     void __attribute__((weak)) SVC_Handler                (void)	 { Default_Handler(); }
     void __attribute__((weak)) DebugMon_Handler           (void)	 { Default_Handler(); }
     void __attribute__((weak)) PendSV_Handler             (void)	 { Default_Handler(); }
     void __attribute__((weak)) SysTick_Handler            (void)	 { Default_Handler(); }
     void __attribute__((weak)) WWDG_IRQHandler            (void)	 { Default_Handler(); }
     void __attribute__((weak)) PVD_IRQHandler             (void)	 { Default_Handler(); }
     void __attribute__((weak)) TAMP_STAMP_IRQHandler      (void)	 { Default_Handler(); }
     void __attribute__((weak)) RTC_WKUP_IRQHandler        (void)	 { Default_Handler(); }
     void __attribute__((weak)) FLASH_IRQHandler           (void)	 { Default_Handler(); }                                        
     void __attribute__((weak)) RCC_IRQHandler             (void)	 { Default_Handler(); }                                         
     void __attribute__((weak)) EXTI0_IRQHandler           (void)	 { Default_Handler(); }
     void __attribute__((weak)) EXTI1_IRQHandler           (void)	 { Default_Handler(); }
     void __attribute__((weak)) EXTI2_IRQHandler           (void)	 { Default_Handler(); }
     void __attribute__((weak)) EXTI3_IRQHandler           (void)	 { Default_Handler(); }
     void __attribute__((weak)) EXTI4_IRQHandler           (void)	 { Default_Handler(); }
     void __attribute__((weak)) DMA1_Channel1_IRQHandler   (void)	 { Default_Handler(); }
     void __attribute__((weak)) DMA1_Channel2_IRQHandler   (void)	 { Default_Handler(); }
     void __attribute__((weak)) DMA1_Channel3_IRQHandler   (void)	 { Default_Handler(); }
     void __attribute__((weak)) DMA1_Channel4_IRQHandler   (void)	 { Default_Handler(); }
     void __attribute__((weak)) DMA1_Channel5_IRQHandler   (void)	 { Default_Handler(); }
     void __attribute__((weak)) DMA1_Channel6_IRQHandler   (void)	 { Default_Handler(); }
     void __attribute__((weak)) DMA1_Channel7_IRQHandler   (void)	 { Default_Handler(); }
     void __attribute__((weak)) ADC1_2_IRQHandler          (void)	 { Default_Handler(); }
     void __attribute__((weak)) USB_HP_CAN1_TX_IRQHandler  (void)	 { Default_Handler(); }
	 void __attribute__((weak)) USB_LP_CAN1_RX0_IRQHandler (void)	 { Default_Handler(); }
	 void __attribute__((weak)) CAN1_RX1_IRQHandler        (void)	 { Default_Handler(); }
	 void __attribute__((weak)) CAN1_SCE_IRQHandler        (void)	 { Default_Handler(); }
	 void __attribute__((weak)) EXTI9_5_IRQHandler         (void)	 { Default_Handler(); }
	 void __attribute__((weak)) TIM1_BRK_IRQHandler        (void)	 { Default_Handler(); }
	 void __attribute__((weak)) TIM1_UP_IRQHandler         (void)	 { Default_Handler(); }
	 void __attribute__((weak)) TIM1_TRG_COM_IRQHandler    (void)	 { Default_Handler(); }
	 void __attribute__((weak)) TIM1_CC_IRQHandler         (void)	 { Default_Handler(); }
	 void __attribute__((weak)) TIM2_IRQHandler            (void)	 { Default_Handler(); }
	 void __attribute__((weak)) TIM3_IRQHandler            (void)	 { Default_Handler(); }
	 void __attribute__((weak)) TIM4_IRQHandler            (void)	 { Default_Handler(); }
	 void __attribute__((weak)) I2C1_EV_IRQHandler         (void)	 { Default_Handler(); }
	 void __attribute__((weak)) I2C1_ER_IRQHandler         (void)	 { Default_Handler(); }
	 void __attribute__((weak)) I2C2_EV_IRQHandler         (void)	 { Default_Handler(); }
	 void __attribute__((weak)) I2C2_ER_IRQHandler         (void)	 { Default_Handler(); }
	 void __attribute__((weak)) SPI1_IRQHandler            (void)	 { Default_Handler(); }
	 void __attribute__((weak)) SPI2_IRQHandler            (void)	 { Default_Handler(); }
	 void __attribute__((weak)) USART1_IRQHandler          (void)	 { Default_Handler(); }
	 void __attribute__((weak)) USART2_IRQHandler          (void)	 { Default_Handler(); }
	 void __attribute__((weak)) USART3_IRQHandler          (void)	 { Default_Handler(); }
	 void __attribute__((weak)) EXTI15_10_IRQHandler       (void)	 { Default_Handler(); }
	 void __attribute__((weak)) RTC_Alarm_IRQHandler       (void)	 { Default_Handler(); }
	 void __attribute__((weak)) USBWakeUp_IRQHandler       (void)	 { Default_Handler(); }                                                          


/********************************* vector table ********************************/
uint32_t* Vector_Table[] __attribute__((section(".isr_vector")))={
    (uint32_t *)MSP,     /*Main stack pointer*/
    (uint32_t *)Reset_Handler,                     /* Reset */
    (uint32_t *)NMI_Handler,                       /* Non maskable interrupt */
    (uint32_t *)HardFault_Handler,                 /* All class of fault */
    (uint32_t *)MemManage_Handler,                 /* Memory management */
    (uint32_t *)BusFault_Handler,                  /* Pre-fetch fault, memory access fault */ 
    (uint32_t *)UsageFault_Handler,                /* Undefined instruction or illegal state */
    0,
    0,
    0,
    0,
    (uint32_t *)SVC_Handler,                       /* System service call via SWI instruction */
    (uint32_t *)DebugMon_Handler,                  /* Debug Monitor */
    0,
    (uint32_t *)PendSV_Handler,                    /* Pendable request for system service */ 
    (uint32_t *)SysTick_Handler,
    (uint32_t *)WWDG_IRQHandler,                   /* Window WatchDog              */                                        
    (uint32_t *)PVD_IRQHandler,                    /* PVD through EXTI Line detection */                        
    (uint32_t *)TAMP_STAMP_IRQHandler,             /* Tamper and TimeStamps through the EXTI line */            
    (uint32_t *)RTC_WKUP_IRQHandler,               /* RTC Wakeup through the EXTI line */                      
    (uint32_t *)FLASH_IRQHandler,                  /* FLASH                        */                                          
    (uint32_t *)RCC_IRQHandler,                    /* RCC                          */                                            
    (uint32_t *)EXTI0_IRQHandler,                  /* EXTI Line0                   */                        
    (uint32_t *)EXTI1_IRQHandler,                  /* EXTI Line1                   */                          
    (uint32_t *)EXTI2_IRQHandler,                  /* EXTI Line2                   */                          
    (uint32_t *)EXTI3_IRQHandler,                  /* EXTI Line3                   */                          
    (uint32_t *)EXTI4_IRQHandler,                  /* EXTI Line4                   */                          
    (uint32_t *)DMA1_Channel1_IRQHandler,          /* DMA1 Channel1 global interrupt */                  
    (uint32_t *)DMA1_Channel2_IRQHandler,          /* DMA1 Channel2 global interrupt */                   
    (uint32_t *)DMA1_Channel3_IRQHandler,          /* DMA1 Channel3 global interrupt */                   
    (uint32_t *)DMA1_Channel4_IRQHandler,          /* DMA1 Channel4 global interrupt */                   
    (uint32_t *)DMA1_Channel5_IRQHandler,          /* DMA1 Channel5 global interrupt */                   
    (uint32_t *)DMA1_Channel6_IRQHandler,          /* DMA1 Channel6 global interrupt */                   
    (uint32_t *)DMA1_Channel7_IRQHandler,          /* DMA1 Channel7 global interrupt */                   
    (uint32_t *)ADC1_2_IRQHandler,                 /* ADC1 and ADC2 global interrupt */     
    (uint32_t *) USB_HP_CAN1_TX_IRQHandler,        /*USB high priority or CAN TX interrupts*/
	(uint32_t *) USB_LP_CAN1_RX0_IRQHandler,       /*USB low priority or CAN RX0 interrupts*/
	(uint32_t *) CAN1_RX1_IRQHandler,               /*CAN RX1 interrupt*/
	(uint32_t *) CAN1_SCE_IRQHandler,               /*CAN SCE interrupt*/
	(uint32_t *) EXTI9_5_IRQHandler ,               /*EXTI Line[9:5] interrupts*/
	(uint32_t *) TIM1_BRK_IRQHandler,               /*TIM1 Break interrupt and TIM9 global interrupt*/
	(uint32_t *) TIM1_UP_IRQHandler ,               /*TIM1 Update interrupt and TIM10 global interrupt*/
	(uint32_t *) TIM1_TRG_COM_IRQHandler,           /*TIM1 Trigger and Commutation interrupts and TIM11 global interrupt*/
	(uint32_t *) TIM1_CC_IRQHandler,                /*TIM1 Capture Compare interrupt*/
	(uint32_t *) TIM2_IRQHandler,                   /*TIM2 global interrupt*/
	(uint32_t *) TIM3_IRQHandler,                   /*TIM3 global interrupt*/
	(uint32_t *) TIM4_IRQHandler,                   /*TIM4 global interrupt*/
	(uint32_t *) I2C1_EV_IRQHandler,                /*I2C1 event interrupt*/
	(uint32_t *) I2C1_ER_IRQHandler,                /*I2C1 error interrupt*/
	(uint32_t *) I2C2_EV_IRQHandler,                /*I2C2 event interrupt*/
	(uint32_t *) I2C2_ER_IRQHandler,                /*I2C2 error interrupt*/
	(uint32_t *) SPI1_IRQHandler,                   /*SPI1 global interrupt*/
	(uint32_t *) SPI2_IRQHandler,                   /*SPI2 global interrupt*/
	(uint32_t *) USART1_IRQHandler,                 /*USART1 global interrupt*/
	(uint32_t *) USART2_IRQHandler,                 /*USART2 global interrupt*/
	(uint32_t *) USART3_IRQHandler,                 /*USART3 global interrupt*/
	(uint32_t *) EXTI15_10_IRQHandler,              /*EXTI Line[15:10] interrupts*/
	(uint32_t *) RTC_Alarm_IRQHandler,              /*RTC alarm through EXTI line interrupt*/
	(uint32_t *) USBWakeUp_IRQHandler,              /*USB wakeup from suspend through EXTI line interrupt*/
	 0,                                    
	 0,                                
	 0,                                
	 0,                                 
	 0,                                 
	 0,                                
	 0                             
};