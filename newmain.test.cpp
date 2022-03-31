#include <gtest/gtest.h>
#include "sg2Pro.X/newmain.c"
#include "sg2Pro.X/PinsCheck.h"
#include "sg2Pro.X/EngBinSt.h"

class EngBinStateTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    TEngTemp = 0xd6;
    UbattMin = 0x93;
    UbattMinLow = 0x80;

    sec = 0;
    min = 0;
    hour = 0;
    day = 0;
    addrh = 0;
    addrl = 0x0;
    EngSt = OFF;
    BinSt = OFF;

    ASRONTmr = 0xFF;
    NewBinTmr = 0xFF;
    NewASRTmr = 0xFF;
    EngStartTmr = 0xFF;
    CN25Tmr = 0xFF;

    UbattTmr = 0xFF;
    IntSvcSync = 0;
    StateTest = ST_NONE;
    CN21CN25State = CN_2125NONE;

    CheckedNow = CN_NONE;
    PinState = (1 << CN_21) | (1 << CN_25);
    NewState = 0;
  }
};

TEST_F(EngBinStateTest, Error_Should_Be_Reset_If_Eng_is_ON)
{
  EngSt = ERR_BINPATTERN;
  BinSt = ERR_ENGPATTERN;
  PinState = (1 << CN_210) | (1 << CN_23); // Eng is on
  EngBinState();
  // Engine will be ON
  EXPECT_EQ(EngSt, ENGON);
  EXPECT_EQ(BinSt, OFF);
}

// Second button (CN_25) double click: Start eng/bin sequence

TEST_F(EngBinStateTest, Button_2_pushed_LOW)
{
  EngSt = OFF;
  BinSt = OFF;
  PinState = (1 << CN_21) | (0 << CN_25); // Start command sent
  TEB = 0x80;                             // More than 0x7F
  CN21CN25State = CN_2125NONE;
  EngBinState();
  EXPECT_EQ(CN21CN25State, CN_25FIRSTHIGH);
  EXPECT_EQ(CN25Tmr, 0x4);
}

TEST_F(EngBinStateTest, Button_2_pushed_LOW_When_Eng_Or_Bin_In_Err_State)
{
  EngSt = ERR_BINPATTERN;
  BinSt = ERR_ENGPATTERN;
  PinState = (1 << CN_21) | (0 << CN_25); // Start command sent
  TEB = 0x80;                             // More than 0x7F
  CN21CN25State = CN_2125NONE;
  EngBinState();
  EXPECT_EQ(CN21CN25State, CN_25FIRSTHIGH);
  EXPECT_EQ(CN25Tmr, 0x4);
}

TEST_F(EngBinStateTest, Button_2_gets_HIGH_After_Being_Low)
{
  PinState = (1 << CN_21) | (1 << CN_25);
  CN21CN25State = CN_25FIRSTHIGH;
  EngBinState();
  EXPECT_EQ(CN21CN25State, CN_25FIRSTLOW);
  EXPECT_EQ(CN25Tmr, 0x4);
}

TEST_F(EngBinStateTest, Button_2_Second_Push_Engine_To_Start)
{
  PinState = (1 << CN_21) | (0 << CN_25);
  CN21CN25State = CN_25FIRSTLOW;
  TEB = 0x80; // Engine start

  EngBinState();
  EXPECT_EQ(CN21CN25State, CN_2125NONE);
  EXPECT_EQ(EngSt, ASRTOST);
  EXPECT_EQ(NewASRTmr, 0x4);
}

TEST_F(EngBinStateTest, Button_2_Second_Push_Bin_To_Start)
{
  PinState = (1 << CN_21) | (0 << CN_25);
  CN21CN25State = CN_25FIRSTLOW;
  TEB = 0x70; // Binar start

  EngBinState();
  EXPECT_EQ(CN21CN25State, CN_2125NONE);
  EXPECT_EQ(BinSt, BINTOST);
  EXPECT_EQ(NewBinTmr, 0x4);
}

TEST_F(EngBinStateTest, Button_2_Second_HIGH_Engine_To_Start)
{
  PinState = (1 << CN_21) | (1 << CN_25);
  EngSt = ASRTOST;
  DO26 = 1;
  EngBinState();
  EXPECT_EQ(EngSt, ASROFFON);
  EXPECT_EQ(NewASRTmr, 0x4);
  EXPECT_EQ(DO26, 0);
}

TEST_F(EngBinStateTest, Button_2_Second_HIGH_Binar_To_Start)
{
  PinState = (1 << CN_21) | (1 << CN_25);
  BinSt = BINTOST;
  DO1819 = 0;
  EngBinState();
  EXPECT_EQ(BinSt, BINOFFON);
  EXPECT_EQ(NewBinTmr, 0x4);
  EXPECT_EQ(DO1819, 1);
}

// Engine start tests

TEST_F(EngBinStateTest, ASROFFON_to_ENGASRON)
{
  PinState |= (1 << CN_210);  // ASR is ON CN_23 == 0
  EngSt = ASROFFON;
  EngBinState();
  EXPECT_EQ(CN25Tmr, 0xFF);
  EXPECT_EQ(CN21CN25State, CN_2125NONE);
  EXPECT_EQ(ASRONTmr, 0xFF);
  EXPECT_EQ(EngSt, ENGASRON);
  EXPECT_EQ(NewASRTmr, 0x5);
}

TEST_F(EngBinStateTest, ASROFFON_to_ENGON_Key_Is_Inserted)
{
  PinState |= (1 << CN_210) | (1 << CN_23);  // ASR is OFF CN_23 == 1
  EngSt = ASROFFON;
  EngBinState();
  EXPECT_EQ(CN25Tmr, 0xFF);
  EXPECT_EQ(CN21CN25State, CN_2125NONE);
  EXPECT_EQ(ASRONTmr, 0xFF);
  EXPECT_EQ(EngSt, ENGON);
  EXPECT_EQ(NewASRTmr, 0x3);
}

TEST_F(EngBinStateTest, Eng_turned_on_when_EngStartTmr_hasn_t_been_expired)
{
  PinState |= (1 << CN_210) | (1 << CN_23);  // ASR is OFF CN_23 == 1
  EngSt = OFF;
  EngStartTmr = 0x1;
  EngBinState();
  EXPECT_EQ(CN25Tmr, 0xFF);
  EXPECT_EQ(CN21CN25State, CN_2125NONE);
  EXPECT_EQ(ASRONTmr, 0xFF);
  EXPECT_EQ(EngSt, ENGON);
  EXPECT_EQ(NewASRTmr, 0xFF);
}

TEST_F(EngBinStateTest, Engine_turned_OFF_when_started_by_ENGASRON)
{
  PinState |= (0 << CN_210);  // IG is OFF
  EngSt = ENGASRON;
  EngBinState();
  EXPECT_EQ(EngStartTmr, 0x10);
  EXPECT_EQ(EngSt, OFF);
}

// --------------------------------------
// Engine start by ASRONTmr.
// This timer expire when binar just worked
// --------------------------------------

TEST_F(EngBinStateTest, Engine_should_be_started_by_ASRONTmr)
{
  EngSt = OFF;
  BinSt = OFF;
  ASRONTmr = 0;
  DO26 = 1;
  EngBinState();
  EXPECT_EQ(ASRONTmr, 0xFF);
  EXPECT_EQ(NewASRTmr, 0x4);
  EXPECT_EQ(EngSt, ASROFFON);
  EXPECT_EQ(DO26, 0);
}

TEST_F(EngBinStateTest, Engine_should_NOT_be_started__when_Eng_OR_Bin_are_not_OFF)
{
  // 1. Eng is ON
  ASRONTmr = 0;
  EngSt = ASROFFON;
  EngBinState();
  EXPECT_EQ(ASRONTmr, 0xFF);
  EXPECT_EQ(EngSt, ASROFFON);
  EngSt = OFF;
  // 2. BinSt is ON
  ASRONTmr = 0;
  BinSt = BINOFFON;
  EngBinState();
  EXPECT_EQ(ASRONTmr, 0xFF);
  EXPECT_EQ(EngSt, OFF);
}

// --------------------------------------
// Engine start by UbattTmr.
// This timer expire when Ubatt become too low
// --------------------------------------

TEST_F(EngBinStateTest, Engine_should_be_started_by_Low_Ubatt)
{
  EngSt = OFF;
  BinSt = OFF;
  UbattTmr = UBT_TMRST; // UbattTmr is set to UBT_TMRST when Ubatt goes to Low threshold
  DO26 = 1;
  TEB = 0x80; // Engine will start
  EngBinState();
  EXPECT_EQ(NewASRTmr, 0x4);
  EXPECT_EQ(EngSt, ASROFFON);
  EXPECT_EQ(DO26, 0);
}

TEST_F(EngBinStateTest, Eng_start_should_be_skipped_by_low_voltage)
{
  EngSt = OFF;
  UbattTmr = UBT_TMRST; // UbattTmr is set to UBT_TMRST when Ubatt goes to Low threshold
  TEB = 0x80; // Engine will start
  // 1. Binar is already ON
  BinSt = BINON;
  EngBinState();
  EXPECT_EQ(EngSt, OFF);
  BinSt = OFF;
  // 2. ASRONTmr is running (not 0xFF)
  ASRONTmr = 0x03;
  EngBinState();
  EXPECT_EQ(EngSt, OFF);
  ASRONTmr = 0xFF;
  // 3. NewBinTmr is not 0xFF
  NewBinTmr = 0x03;
  EngBinState();
  EXPECT_EQ(EngSt, OFF);
  NewBinTmr = 0xFF;
  // 4. NewASRTmr is not 0xFF
  NewASRTmr = 0x03;
  EngBinState();
  EXPECT_EQ(EngSt, OFF);
  NewASRTmr = 0xFF;
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}