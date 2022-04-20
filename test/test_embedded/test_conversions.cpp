/**
 * @file test_conversions.cpp
 * @author Markus Bäcker (markus.baecker@ovgu.de)
 * @brief 
 * @version 0.1
 * @date 2022-02-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/

#include <Arduino.h>
#include "ADS1299.hh"
#include <unity.h>

ADS1299 ADStest;

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

void test_function_hextovolt_conversion(void) {
    float LSB = 1000 * (4.5) / (8388608);
    TEST_ASSERT_EQUAL(100.0, ADStest.convertHEXtoVolt(0x147AE1));
    TEST_ASSERT_EQUAL(-200.0, ADStest.convertHEXtoVolt( 0xD70A3D));
}

void blink();


void setup() {
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_function_hextovolt_conversion);

    UNITY_END();
}

void loop() {
// indicate that testsoftware upload was sucessfull
blink();

}

void blink(){
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(500);
}