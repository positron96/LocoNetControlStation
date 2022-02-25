
#include "LocoSpeed.h"

#include <stdio.h>
#include <unity.h>

void testLocoSpeed() {

    for(size_t a = 0; a<LocoSpeed::MAX_SPEED; a++) {
        LocoSpeed s = LocoSpeed::from128(a);
        TEST_ASSERT_TRUE(s.get128()==a);
    }

    TEST_ASSERT_TRUE(LocoSpeed::fromFloat(-1)== SPEED_EMGR);
    TEST_ASSERT_TRUE(LocoSpeed::fromFloat(0)== SPEED_IDLE);
    TEST_ASSERT_EQUAL(LocoSpeed::fromFloat(1).get128(), LocoSpeed::MAX_SPEED);
    TEST_ASSERT_TRUE(LocoSpeed::fromFloat(0.5).get128() == LocoSpeed::MAX_SPEED/2);

}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(testLocoSpeed);
    UNITY_END();

    return 0;
}
