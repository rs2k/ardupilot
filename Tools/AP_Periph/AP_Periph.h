#include <AP_HAL/AP_HAL.h>
#include "ch.h"

class AP_Periph_FW {
public:
    void init();
    void update();

private:
    void can_start();
    void can_update();
};
