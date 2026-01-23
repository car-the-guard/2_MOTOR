#include "timestamp.h"
#include <sal_api.h>

void TIMESTAMP_get_ms(uint32_t *ms)
{
    if (ms != 0)
    {
        SAL_GetTickCount(ms);
    }
}