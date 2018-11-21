#include "AudioFileSourceSPIFFS.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2SDAC.h"

const int SteamOutputPin = 14;  //D5   on node mcu Used here for Steam output

bool TimeToChuff(long ChuffQuarterPeriod);
void SetUpChuff(void);
void BeginPlay(const char *wavfilename);
void Chuff(void);
void AudioLoop(int SoundToPlay);
bool SoundEffectPlaying(void);

/*/ uses RX, D8, D5 and D4 pins.

  static const uint8_t D4   = 2;  and Blue Led on SP8266
  static const uint8_t D5   = 14;
  static const uint8_t D8   = 15;
*/
  


/*expected wav files are:
 Initiate sound:
   /initiated.wav
 
 CHUFFS:
  /ch1.wav
  /ch2.wav
  /ch3.wav
  /ch4.wav
Whistle:
  /whistle.wav
Brake Squeal
  /brakes.wav




*/
