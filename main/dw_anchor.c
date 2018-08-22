#include "dw_anchor.h"
#include "nrf_deca.h"

void beacon_timeout_handler()
{
    deca_twr_responder_send_rtls_beacon();
}

