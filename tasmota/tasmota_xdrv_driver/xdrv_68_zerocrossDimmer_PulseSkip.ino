/*
  xdrv_68_zerocrossdimmer_PulseSkip.ino - Zero-Cross Dimmer support for Tasmota

  Copyright (C) 2023  Tom Lawton, modified from Stefan Bode's zerocrossdimmer ()

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

#ifdef USE_AC_ZERO_CROSS_DIMMER_PULSESKIP
#warning "Using ZCPulseSkip Driver in place of ZC Dimmer"
/*********************************************************************************************\
 * Zero-Cross AC Dimmer PMM 1..xx use 
\*********************************************************************************************/

#define XDRV_68             68

static const uint16_t MAX_HALFCYCLES = 10; // max halfcycles (100Hz/120Hz) for calculating dimming (reduce flicker)
//static const uint8_t MIN_PERCENT = 5;
//static const uint8_t MAX_PERCENT = 99;


struct AC_ZERO_CROSS_DIMMER_PS {
    uint16_t halfcycles[4] = {0,0,0,0}; // 0 = off, 1 = on, 2 = off, 3 = on
};


struct AC_ZERO_CROSS_DIMMER {
  bool     pins_setup = false;   // verification of pin setup
  bool     dimmer_in_use = false;            // Check if interrupt has to be run. Is stopped if all lights off
  AC_ZERO_CROSS_DIMMER_PS psm_halfcycles[MAX_PWMS];
  uint16_t halfcycle_phase_count[MAX_PWMS];
  uint8_t  current_phase[MAX_PWMS];
  uint16_t level[MAX_PWMS];              

} ac_zero_cross_dimmer;


void IRAM_ATTR ACDimmerZeroCross(uint32_t time) {
  // called from xns_01_counter.ino when counter4 fires

  ac_zero_cross_dimmer.dimmer_in_use = false;
  for (uint8_t i = 0; i < MAX_PWMS; i++) {
    if (Pin(GPIO_PWM1, i) == -1) continue;
    if (ac_zero_cross_dimmer.psm_halfcycles[i].halfcycles[0] == 0) continue;
    ac_zero_cross_dimmer.dimmer_in_use = true;
    if ((++ac_zero_cross_dimmer.halfcycle_phase_count[i]) >= ac_zero_cross_dimmer.psm_halfcycles[i].halfcycles[ac_zero_cross_dimmer.current_phase[i]]) {
      ac_zero_cross_dimmer.halfcycle_phase_count[i] = 0;
      do {
        if ((ac_zero_cross_dimmer.current_phase[i]++) > 3) {
          ac_zero_cross_dimmer.current_phase[i] = 0;
          break;
        }
      } while ((ac_zero_cross_dimmer.psm_halfcycles[i].halfcycles[ac_zero_cross_dimmer.current_phase[i]]) == 0);
          digitalWrite(Pin(GPIO_PWM1, i), ((ac_zero_cross_dimmer.current_phase[i] & 1) == 1) ? HIGH : LOW);
    }
  }
}

void ACDimmerControl() {
  for (uint8_t i = 0; i < MAX_PWMS; i++) {
    if (Pin(GPIO_PWM1, i) == -1)
          continue;
    if (ac_zero_cross_dimmer.level[i] != (Light.fade_running ? Light.fade_cur_10[i] : Light.fade_start_10[i])) {
          pinMode(Pin(GPIO_PWM1, i), OUTPUT);
          AddLog(LOG_LEVEL_INFO, PSTR("ZCD: Zero-CrossDimmer-PSM Pin %d set"), Pin(GPIO_PWM1, i));
          ACDimmerControl_SetPhases(i);
    }
  }
}

void ACDimmerControl_SetPhases(uint8_t index) {
  ac_zero_cross_dimmer.level[index]= Light.fade_running ? Light.fade_cur_10[index] : Light.fade_start_10[index];
  ac_zero_cross_dimmer.current_phase[index]=0;
  ac_zero_cross_dimmer.halfcycle_phase_count[index]=0;
  ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[0]=0;
  uint16_t bestmatch_offhalfcycles=1;
  uint16_t bestmatch_onhalfcycles=0;
  uint16_t test_off=0;
  if (ac_zero_cross_dimmer.level[index]==0) {
    digitalWrite(Pin(GPIO_PWM1, index), LOW);
    return;
  }
  for (uint16_t i=2; i<=MAX_HALFCYCLES; i=i+2) {
    test_off=(i*(1023-ac_zero_cross_dimmer.level[index])+(ac_zero_cross_dimmer.level[index]/2))/ac_zero_cross_dimmer.level[index];
    if ((test_off+i)>MAX_HALFCYCLES) continue;
    if (abs(i*1023/(i+test_off)-ac_zero_cross_dimmer.level[index])<abs(bestmatch_onhalfcycles*1023/(bestmatch_onhalfcycles+bestmatch_offhalfcycles)-ac_zero_cross_dimmer.level[index])) {
      bestmatch_offhalfcycles=test_off;
      bestmatch_onhalfcycles=i;
    }
  }
  if (bestmatch_onhalfcycles==0){
    // closest match is just off
    digitalWrite(Pin(GPIO_PWM1, index), LOW);
    return;
  }
  if (bestmatch_offhalfcycles==0){
    // closest match is just on
    digitalWrite(Pin(GPIO_PWM1, index), HIGH);
    return;
  }
  ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[0]=bestmatch_offhalfcycles;
  ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[1]=bestmatch_onhalfcycles;
  ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[2]=0;
  ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[3]=0;

  if (bestmatch_onhalfcycles>2 && bestmatch_offhalfcycles>1) {
    // split up
    ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[3]=(bestmatch_onhalfcycles>>2)*2;
    ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[1]=bestmatch_onhalfcycles-ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[3];
    ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[2]=(bestmatch_offhalfcycles>>1);
    ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[0]=bestmatch_offhalfcycles-ac_zero_cross_dimmer.psm_halfcycles[index].halfcycles[2];
  }
}


void ACDimmerLogging(void)
{

    AddLog(LOG_LEVEL_DEBUG, PSTR("ZCD-PSM: ZeroEnable %d -> %d"),
      ac_zero_cross_dimmer.dimmer_in_use, ac_zero_cross_dimmer.pins_setup);
    for (uint8_t i = 0; i < MAX_PWMS; i++){
      if (Pin(GPIO_PWM1, i) == -1) continue;
       AddLog(LOG_LEVEL_DEBUG, PSTR("ZCD: PWM[%d] %d %d %d %d - %d  %d - %d"), 
        i+1, ac_zero_cross_dimmer.psm_halfcycles[i].halfcycles[0], ac_zero_cross_dimmer.psm_halfcycles[i].halfcycles[1], ac_zero_cross_dimmer.psm_halfcycles[i].halfcycles[2],ac_zero_cross_dimmer.psm_halfcycles[i].halfcycles[3], ac_zero_cross_dimmer.current_phase[i],  ac_zero_cross_dimmer.halfcycle_phase_count[i], ac_zero_cross_dimmer.level[i]
      );
    }
} 

#ifdef USE_WEBSERVER
/* #ifdef ZCDIMMERSET_SHOW
void ACDimmerShow(void)
{
  char c_ZCDimmerSetBuffer[8];
  for (uint8_t i = 0; i < MAX_PWMS; i++){
    if (Pin(GPIO_PWM1, i) == -1) continue;
    if (ac_zero_cross_dimmer.detailpower[i]){
      dtostrfd(ac_zero_cross_dimmer.detailpower[i]/100.0, 2, c_ZCDimmerSetBuffer);
      WSContentSend_PD(PSTR("{s}ZCDimmer%d{m}%s %%{e}"), i+1, c_ZCDimmerSetBuffer);
    }
  }
}
#endif  // ZCDIMMERSET_SHOW */
#endif  // USE_WEBSERVER


bool Xdrv68(uint32_t function)
{
  bool result = false;
  if (Settings->flag4.zerocross_dimmer) {
    switch (function) {
//       case FUNC_INIT:
//         break;
      case FUNC_EVERY_SECOND:
        ACDimmerControl();
        ACDimmerLogging();
        break;

    //   case FUNC_COMMAND:
    //     result = DecodeCommand(kZCDimmerCommands, ZCDimmerCommand);
    //     break; 
#ifdef USE_WEBSERVER
/* #ifdef ZCDIMMERSET_SHOW
      case FUNC_WEB_SENSOR:
        ACDimmerShow();
        break;
#endif  // ZCDIMMERSET_SHOW */
#endif  // USE_WEBSERVER        
    }
  }
  return result;
}
#endif  // USE_AC_ZERO_CROSS_DIMMER_PULSESKIP
