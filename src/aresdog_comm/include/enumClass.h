/**********************************************************************
 Copyright (c) 2023-2024, Ares Robotics. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class UserCommand{
    NONE,
    START,      // trotting
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // freeStand
    L2_Y,       // move_base
    L1_X,       // balanceTest
    L1_A,       // swingTest
    L1_Y        // stepTest
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,
    MOVE_BASE,
    BALANCETEST,
    SWINGTEST,
    STEPTEST
};

#endif  // ENUMCLASS_H 