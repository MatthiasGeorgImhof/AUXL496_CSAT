#ifdef __x86_64__

#include "mock_hal/mock_hal_rtc.h"
#include <stdio.h>  // For debugging/default implementations
#include <string.h> // For memcpy

//------------------------------------------------------------------------------
//  MOCK RTC STATE VARIABLES - Definitions
//------------------------------------------------------------------------------

// Variables to store the mocked RTC time and date (same as before)
RTC_TimeTypeDef mocked_rtc_time = {
    .Hours = 0,
    .Minutes = 0,
    .Seconds = 0,
    .TimeFormat = 0,
    .SubSeconds = 0,
    .SecondFraction = 0,
    .DayLightSaving = 0,
    .StoreOperation = 0
};  // Initialize to zero

RTC_DateTypeDef mocked_rtc_date = {
    .WeekDay = 0,
    .Month = 0,
    .Date = 0,
    .Year = 0
};  // Initialize to zero

//Status of the set and get
HAL_StatusTypeDef mocked_rtc_set_status = HAL_OK; //Default status to ok
HAL_StatusTypeDef mocked_rtc_get_status = HAL_OK; //Default status to ok

//synchro shift variables
uint32_t mocked_synchro_shift_add1s = 0;
uint32_t mocked_synchro_shift_subfs = 0;
HAL_StatusTypeDef mocked_rtc_ex_set_synchro_shift_status = HAL_OK;

//------------------------------------------------------------------------------
//  MOCK RTC HELPER FUNCTION IMPLEMENTATIONS (Getters and Setters)
//------------------------------------------------------------------------------

void set_mocked_rtc_time(RTC_TimeTypeDef time) {
    mocked_rtc_time = time; //Direct assignment
}

void set_mocked_rtc_date(RTC_DateTypeDef date) {
    mocked_rtc_date = date; //Direct assignment
}

void set_mocked_rtc_set_status(HAL_StatusTypeDef status){
    mocked_rtc_set_status = status;
}
//Setters for mocked HAL status for Get functions
void set_mocked_rtc_get_status(HAL_StatusTypeDef status){
    mocked_rtc_get_status = status;
}

//Synchro shift setters
void set_mocked_synchro_shift_add1s(uint32_t add){
    mocked_synchro_shift_add1s = add;
}

void set_mocked_synchro_shift_subfs(uint32_t shift){
    mocked_synchro_shift_subfs = shift;
}

void set_mocked_rtc_ex_set_synchro_shift_status(HAL_StatusTypeDef status){
    mocked_rtc_ex_set_synchro_shift_status = status;
}

// Time and Date Getters (for reading the mocked values) (same as before)
RTC_TimeTypeDef get_mocked_rtc_time(void) {
    return mocked_rtc_time;
}

RTC_DateTypeDef get_mocked_rtc_date(void) {
    return mocked_rtc_date;
}

//synchro shift getters
uint32_t get_mocked_synchro_shift_add1s(void){
    return mocked_synchro_shift_add1s;
}

uint32_t get_mocked_synchro_shift_subfs(void){
    return mocked_synchro_shift_subfs;
}

//Clear the values
void clear_mocked_rtc(){
    memset(&mocked_rtc_time, 0, sizeof(mocked_rtc_time));
    memset(&mocked_rtc_date, 0, sizeof(mocked_rtc_date));
    mocked_rtc_set_status = HAL_OK;
    mocked_rtc_get_status = HAL_OK;
    mocked_synchro_shift_add1s = 0;
    mocked_synchro_shift_subfs = 0;
    mocked_rtc_ex_set_synchro_shift_status = HAL_OK;
}

//------------------------------------------------------------------------------
//  MOCK HAL FUNCTION IMPLEMENTATIONS
//------------------------------------------------------------------------------

HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef * /*hrtc*/) {
    // No internal state to modify for initialization (for this mock)
    printf("HAL_RTC_Init called\n");  // For debugging
    return HAL_OK;
}

HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef * /*hrtc*/) {
    // No internal state to modify for de-initialization (for this mock)
    printf("HAL_RTC_DeInit called\n"); // For debugging
    return HAL_OK;
}

void HAL_RTC_MspInit(RTC_HandleTypeDef * /*hrtc*/) {
    // No MSP initialization needed for the mock
    printf("HAL_RTC_MspInit called\n");  // For debugging
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef * /*hrtc*/) {
    // No MSP de-initialization needed for the mock
    printf("HAL_RTC_MspDeInit called\n"); // For debugging
}

HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef * /*hrtc*/, RTC_TimeTypeDef *sTime, uint32_t /*Format*/) {
    if (sTime != NULL) {
        set_mocked_rtc_time(*sTime); // Store the passed time
        return mocked_rtc_set_status;
    } else {
        return HAL_ERROR; // Handle null pointer
    }
}

HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef * /*hrtc*/, RTC_TimeTypeDef *sTime, uint32_t /*Format*/) {
    if (sTime != NULL) {
        *sTime = get_mocked_rtc_time(); //Return the mocked time
        return mocked_rtc_get_status;
    } else {
        return HAL_ERROR; // Handle null pointer
    }
}

HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef * /*hrtc*/, RTC_DateTypeDef *sDate, uint32_t /*Format*/) {
     if (sDate != NULL) {
        set_mocked_rtc_date(*sDate); // Store the passed date
         return mocked_rtc_set_status;
    } else {
        return HAL_ERROR; // Handle null pointer
    }
}

HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef * /*hrtc*/, RTC_DateTypeDef *sDate, uint32_t /*Format*/) {
    if (sDate != NULL) {
        *sDate = get_mocked_rtc_date(); //Return mocked date
        return mocked_rtc_get_status;
    } else {
        return HAL_ERROR; // Handle null pointer
    }
}

// Implement HAL_RTCEx_SetSynchroShift
HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(RTC_HandleTypeDef */*hrtc*/, uint32_t ShiftAdd1S, uint32_t ShiftSubFS) {
        set_mocked_synchro_shift_add1s(ShiftAdd1S); // Store the passed shift
        set_mocked_synchro_shift_subfs(ShiftSubFS); // Store the passed shift
        return mocked_rtc_ex_set_synchro_shift_status;
}

#include <stdint.h>  // For uint8_t

//------------------------------------------------------------------------------
//  RTC_ByteToBcd2
//------------------------------------------------------------------------------
//  Converts a byte (0-99) to BCD format.
//
//  Input:
//      Value: The byte to convert (0-99).
//
//  Output:
//      The BCD representation of the byte.
//------------------------------------------------------------------------------
uint8_t RTC_ByteToBcd2(uint8_t Value) {
    uint8_t bcdhigh = (Value / 10) << 4;
    return bcdhigh | (Value % 10);
}

//------------------------------------------------------------------------------
//  RTC_Bcd2ToByte
//------------------------------------------------------------------------------
//  Converts a BCD byte to standard byte format.
//
//  Input:
//      Value: The BCD byte to convert.
//
//  Output:
//      The standard byte representation of the BCD value.
//------------------------------------------------------------------------------
uint8_t RTC_Bcd2ToByte(uint8_t Value) {
    uint8_t high = (Value >> 4) * 10;
    return high + (Value & 0x0F);
}

#endif /*__x86_64__*/