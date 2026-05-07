#ifndef LOADING_H
#define LOADING_H

#include <Arduino.h>

void loadingDraw(uint8_t progress, uint16_t angleDeg);
void loadingShowStage(uint8_t progress, unsigned long minStageMs = 120);

#endif
