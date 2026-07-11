/** Unity runner for the ota_manager host suite (pure session layer). */
#include <stdlib.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

extern void run_session_tests(void);

void app_main(void)
{
    UNITY_BEGIN();
    run_session_tests();
    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
