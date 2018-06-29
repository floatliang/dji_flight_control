#ifndef PARAMETER_H
#define PARAMETER_H
const int WINDOW_SIZE = 30;
const int NUMBER_OF_AP = 1;
const int MIN_FRAME_CNT = 10;
const int NUMBER_OF_STATE = (WINDOW_SIZE + 1) * 9 + NUMBER_OF_AP * 3;
const int NUMBER_OF_PRIOR = WINDOW_SIZE * 9 + NUMBER_OF_AP * 3;
#endif
