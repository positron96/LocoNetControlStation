
#include "LocoSpeed.h"

#include <stdio.h>
#include <unity.h>

void testLocoSpeed() {

    TEST_ASSERT_EQUAL_MESSAGE(0, SPEED_IDLE.get128(), "IDLE");
    TEST_ASSERT_EQUAL_MESSAGE(1, SPEED_EMGR.get128(), "EMGR");    

    // test 14 speed steps
    for(uint8_t a = 0; a<16; a++) {
        LocoSpeed s = LocoSpeed{a, SpeedMode::S14};
        uint8_t ret = s.getDCC(SpeedMode::S14);
        //printf("in %d (%d) out %d\n", a, s.get128(), ret);
        TEST_ASSERT_EQUAL_MESSAGE(a, ret, "S14" );
    }
    
    // test 28 speed steps
    for(uint8_t a = 0; a<32; a++) {
        LocoSpeed s = LocoSpeed{a, SpeedMode::S28};
        uint8_t ret = s.getDCC(SpeedMode::S28);
        //printf("in %d (%d) out %d\n", a, s.get128(), ret);
        if(a==2) { TEST_ASSERT_MESSAGE(2==ret || 0==ret, "IDLE S28" );
        } else if(a==3) { TEST_ASSERT_MESSAGE(3==ret || 1==ret, "EMGR S28" );
        } else TEST_ASSERT_EQUAL_MESSAGE(a, ret, "S28" );
    }

    // test 128 sppeds, effectively reversibility
    for(size_t a = 0; a<128; a++) {
        LocoSpeed s = LocoSpeed::from128(a);
        TEST_ASSERT_EQUAL_MESSAGE(a, s.get128(), "S128" );
    }

    // test float
    const size_t N=100;
    for(size_t a = 0; a<=N; a++) {
        float in = (float)a/N;
        LocoSpeed s = LocoSpeed::fromFloat(in);
        float out = s.getFloat();
        //printf("in %f (%d) out %f\n", in, s.get128(), out);
        TEST_ASSERT_EQUAL_MESSAGE(a, int(out*N), "Float" );
    }
    TEST_ASSERT_TRUE_MESSAGE(SPEED_EMGR == LocoSpeed::fromFloat(-1), "fromFloat(-1)");
    //TEST_ASSERT_TRUE_MESSAGE(SPEED_IDLE == LocoSpeed::fromFloat(0), "fromFloat(0)");
    //TEST_ASSERT_EQUAL_MESSAGE(127, LocoSpeed::fromFloat(1).get128(), "fromFloat(1)");

}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(testLocoSpeed);
    return UNITY_END();
}
