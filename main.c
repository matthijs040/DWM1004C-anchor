#ifndef STM32L0 // For intellisense.
#define STM32L0
#endif

int main(void)
{
    volatile int number = 5;
    for(int i = 0; i < 10; i++)
        number++;
}