/* Libraries */
#include <iostream>
#include <pigpio.h>

/* Variables and Constants */
using namespace std;
#define LED 17
#define HIGH 0x1
#define LOW 0x0
/* Main function */

int main(void)
{
    cout << "Raspberry Pi initialized!" << endl;
    if (gpioInitialise() < 0) exit(1);

    gpioSetMode(LED, PI_OUTPUT); // 3.3V PIN

    for (;;)
    {
        gpioWrite(LED, HIGH);
        gpioDelay(500000);
        gpioWrite(LED, LOW);
        gpioDelay(500000);
    }

    gpioTerminate();

}
