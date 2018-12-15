/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  AP_Periph main firmware

  To flash this firmware on Linux use:

     st-flash write build/f103-periph/bin/AP_Periph.bin 0x8006000

 */
#include <AP_HAL/AP_HAL.h>
#include "AP_Periph.h"
#include "hal.h"

extern const AP_HAL::HAL &hal;

static AP_Periph_FW periph;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
    periph.init();
}

void loop(void)
{
    periph.update();
}

// usart1 configuration
static const SerialConfig uart1_cfg = {
    57600,   // speed
    0,        // cr1
    0,        // cr2
    0,        // cr3
    nullptr,  // irq_cb
    nullptr,  // ctx
};

// usart2 configuration
static const SerialConfig uart2_cfg = {
    57600,   // speed
    0,        // cr1
    0,        // cr2
    0,        // cr3
    nullptr,  // irq_cb
    nullptr,  // ctx
};

void AP_Periph_FW::init()
{
    sdStart(&SD1, &uart1_cfg);
    sdStart(&SD2, &uart2_cfg);
}


void AP_Periph_FW::update()
{
    static uint32_t xx;
    if (xx++ % 30000 == 0) {
        palToggleLine(HAL_GPIO_PIN_LED);
        chnWrite(&SD1, (const uint8_t *)"uart1", 5);
        chnWrite(&SD2, (const uint8_t *)"uart2", 5);
    }
}

AP_HAL_MAIN();
