--
--  Copyright (C) 2023-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

pragma Restrictions (No_Elaboration_Code);
pragma Ada_2022;

--  with System.Storage_Elements;

package body A0B.MPUXXXX.DMP612 is

   use type A0B.Types.Unsigned_32;

   Firmware_Size : constant := 3_062;

   Firmware_Binary : constant A0B.I2C.Unsigned_8_Array (0 .. Firmware_Size - 1) :=
     (
      --  bank # 0
      16#00#, 16#00#, 16#70#, 16#00#, 16#00#, 16#00#, 16#00#, 16#24#,
      16#00#, 16#00#, 16#00#, 16#02#, 16#00#, 16#03#, 16#00#, 16#00#,
      16#00#, 16#65#, 16#00#, 16#54#, 16#ff#, 16#ef#, 16#00#, 16#00#,
      16#fa#, 16#80#, 16#00#, 16#0b#, 16#12#, 16#82#, 16#00#, 16#01#,
      16#03#, 16#0c#, 16#30#, 16#c3#, 16#0e#, 16#8c#, 16#8c#, 16#e9#,
      16#14#, 16#d5#, 16#40#, 16#02#, 16#13#, 16#71#, 16#0f#, 16#8e#,
      16#38#, 16#83#, 16#f8#, 16#83#, 16#30#, 16#00#, 16#f8#, 16#83#,
      16#25#, 16#8e#, 16#f8#, 16#83#, 16#30#, 16#00#, 16#f8#, 16#83#,
      16#ff#, 16#ff#, 16#ff#, 16#ff#, 16#0f#, 16#fe#, 16#a9#, 16#d6#,
      16#24#, 16#00#, 16#04#, 16#00#, 16#1a#, 16#82#, 16#79#, 16#a1#,
      16#00#, 16#00#, 16#00#, 16#3c#, 16#ff#, 16#ff#, 16#00#, 16#00#,
      16#00#, 16#10#, 16#00#, 16#00#, 16#38#, 16#83#, 16#6f#, 16#a2#,
      16#00#, 16#3e#, 16#03#, 16#30#, 16#40#, 16#00#, 16#00#, 16#00#,
      16#02#, 16#ca#, 16#e3#, 16#09#, 16#3e#, 16#80#, 16#00#, 16#00#,
--   | D_0_104 GYRO_SF  4 bytes      |
      16#20#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#40#, 16#00#, 16#00#, 16#00#, 16#60#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#0c#, 16#00#, 16#00#, 16#00#, 16#0c#, 16#18#, 16#6e#,
      16#00#, 16#00#, 16#06#, 16#92#, 16#0a#, 16#16#, 16#c0#, 16#df#,
      16#ff#, 16#ff#, 16#02#, 16#56#, 16#fd#, 16#8c#, 16#d3#, 16#77#,
      16#ff#, 16#e1#, 16#c4#, 16#96#, 16#e0#, 16#c5#, 16#be#, 16#aa#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#ff#, 16#ff#, 16#0b#, 16#2b#,
      16#00#, 16#00#, 16#16#, 16#57#, 16#00#, 16#00#, 16#03#, 16#59#,
      16#40#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#1d#, 16#fa#,
      16#00#, 16#02#, 16#6c#, 16#1d#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#3f#, 16#ff#, 16#df#, 16#eb#, 16#00#, 16#3e#, 16#b3#, 16#b6#,
      16#00#, 16#0d#, 16#22#, 16#78#, 16#00#, 16#00#, 16#2f#, 16#3c#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#19#, 16#42#, 16#b5#,
      16#00#, 16#00#, 16#39#, 16#a2#, 16#00#, 16#00#, 16#b3#, 16#65#,
      16#d9#, 16#0e#, 16#9f#, 16#c9#, 16#1d#, 16#cf#, 16#4c#, 16#34#,
      16#30#, 16#00#, 16#00#, 16#00#, 16#50#, 16#00#, 16#00#, 16#00#,
      16#3b#, 16#b6#, 16#7a#, 16#e8#, 16#00#, 16#64#, 16#00#, 16#00#,
      16#00#, 16#c8#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      --  bank # 1
      16#10#, 16#00#, 16#00#, 16#00#, 16#10#, 16#00#, 16#fa#, 16#92#,
      16#10#, 16#00#, 16#22#, 16#5e#, 16#00#, 16#0d#, 16#22#, 16#9f#,
      16#00#, 16#01#, 16#00#, 16#00#, 16#00#, 16#32#, 16#00#, 16#00#,
      16#ff#, 16#46#, 16#00#, 16#00#, 16#63#, 16#d4#, 16#00#, 16#00#,
      16#10#, 16#00#, 16#00#, 16#00#, 16#04#, 16#d6#, 16#00#, 16#00#,
      16#04#, 16#cc#, 16#00#, 16#00#, 16#04#, 16#cc#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#10#, 16#72#, 16#00#, 16#00#, 16#00#, 16#40#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#06#, 16#00#, 16#02#, 16#00#, 16#05#, 16#00#, 16#07#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#64#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#05#,
      16#00#, 16#05#, 16#00#, 16#64#, 16#00#, 16#20#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#40#, 16#00#, 16#00#, 16#00#, 16#03#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#32#, 16#f8#, 16#98#, 16#00#, 16#00#,
      16#ff#, 16#65#, 16#00#, 16#00#, 16#83#, 16#0f#, 16#00#, 16#00#,
      16#ff#, 16#9b#, 16#fc#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#10#, 16#00#,
      16#40#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#06#,
      16#00#, 16#00#, 16#b2#, 16#6a#, 16#00#, 16#02#, 16#00#, 16#00#,
      16#00#, 16#01#, 16#fb#, 16#83#, 16#00#, 16#68#, 16#00#, 16#00#,
      16#00#, 16#d9#, 16#fc#, 16#00#, 16#7c#, 16#f1#, 16#ff#, 16#83#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#65#, 16#00#, 16#00#,
      16#00#, 16#64#, 16#03#, 16#e8#, 16#00#, 16#64#, 16#00#, 16#28#,
      16#00#, 16#00#, 16#00#, 16#25#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#16#, 16#a0#, 16#00#, 16#00#, 16#00#, 16#00#, 16#10#, 16#00#,
      16#00#, 16#00#, 16#10#, 16#00#, 16#00#, 16#2f#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#01#, 16#f4#, 16#00#, 16#00#, 16#10#, 16#00#,
      --  bank # 2
      16#00#, 16#28#, 16#00#, 16#00#, 16#ff#, 16#ff#, 16#45#, 16#81#,
      16#ff#, 16#ff#, 16#fa#, 16#72#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#44#, 16#00#, 16#05#,
--                                                   | FIFO_RATE_DIV |
      16#00#, 16#05#, 16#ba#, 16#c6#, 16#00#, 16#47#, 16#78#, 16#a2#,
      16#00#, 16#00#, 16#00#, 16#01#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#06#, 16#00#, 16#00#, 16#00#, 16#00#, 16#14#,
      16#00#, 16#00#, 16#25#, 16#4d#, 16#00#, 16#2f#, 16#70#, 16#6d#,
      16#00#, 16#00#, 16#05#, 16#ae#, 16#00#, 16#0c#, 16#02#, 16#d0#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#1b#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#64#, 16#00#, 16#00#, 16#00#, 16#08#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#1b#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#0e#, 16#00#, 16#0e#,
      16#00#, 16#00#, 16#0a#, 16#c7#, 16#00#, 16#04#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#32#, 16#ff#, 16#ff#, 16#ff#, 16#9c#,
      16#00#, 16#00#, 16#0b#, 16#2b#, 16#00#, 16#00#, 16#00#, 16#02#,
      16#00#, 16#00#, 16#00#, 16#01#, 16#00#, 16#00#, 16#00#, 16#64#,
      16#ff#, 16#e5#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      --  bank # 3
      16#00#, 16#00#, 16#00#, 16#01#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#01#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#01#, 16#80#, 16#00#, 16#00#, 16#01#, 16#80#, 16#00#,
      16#00#, 16#01#, 16#80#, 16#00#, 16#00#, 16#24#, 16#26#, 16#d3#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#06#, 16#00#, 16#10#, 16#00#, 16#96#, 16#00#, 16#3c#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#0c#, 16#0a#, 16#4e#, 16#68#, 16#cd#, 16#cf#, 16#77#, 16#09#,
      16#50#, 16#16#, 16#67#, 16#59#, 16#c6#, 16#19#, 16#ce#, 16#82#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#17#, 16#d7#, 16#84#, 16#00#, 16#03#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#c7#, 16#93#, 16#8f#, 16#9d#, 16#1e#, 16#1b#, 16#1c#, 16#19#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#02#, 16#03#, 16#18#, 16#85#, 16#00#, 16#00#, 16#40#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#03#, 16#00#, 16#00#, 16#00#, 16#03#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#40#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#67#, 16#7d#, 16#df#, 16#7e#,
      16#72#, 16#90#, 16#2e#, 16#55#, 16#4c#, 16#f6#, 16#e6#, 16#88#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#, 16#00#,
      --  bank # 4
      16#d8#, 16#dc#, 16#b4#, 16#b8#, 16#b0#, 16#d8#, 16#b9#, 16#ab#,  --  400
      16#f3#, 16#f8#, 16#fa#, 16#b3#, 16#b7#, 16#bb#, 16#8e#, 16#9e#,
      16#ae#, 16#f1#, 16#32#, 16#f5#, 16#1b#, 16#f1#, 16#b4#, 16#b8#,  --  410
      16#b0#, 16#80#, 16#97#, 16#f1#, 16#a9#, 16#df#, 16#df#, 16#df#,
      16#aa#, 16#df#, 16#df#, 16#df#, 16#f2#, 16#aa#, 16#c5#, 16#cd#,  --  420
      16#c7#, 16#a9#, 16#0c#, 16#c9#, 16#2c#, 16#97#, 16#f1#, 16#a9#,
      16#89#, 16#26#, 16#46#, 16#66#, 16#b2#, 16#89#, 16#99#, 16#a9#,  --  430
      16#2d#, 16#55#, 16#7d#, 16#b0#, 16#b0#, 16#8a#, 16#a8#, 16#96#,
      16#36#, 16#56#, 16#76#, 16#f1#, 16#ba#, 16#a3#, 16#b4#, 16#b2#,  --  440
      16#80#, 16#c0#, 16#b8#, 16#a8#, 16#97#, 16#11#, 16#b2#, 16#83#,
      16#98#, 16#ba#, 16#a3#, 16#f0#, 16#24#, 16#08#, 16#44#, 16#10#,  --  450
      16#64#, 16#18#, 16#b2#, 16#b9#, 16#b4#, 16#98#, 16#83#, 16#f1#,
      16#a3#, 16#29#, 16#55#, 16#7d#, 16#ba#, 16#b5#, 16#b1#, 16#a3#,  --  460
      16#83#, 16#93#, 16#f0#, 16#00#, 16#28#, 16#50#, 16#f5#, 16#b2#,
      16#b6#, 16#aa#, 16#83#, 16#93#, 16#28#, 16#54#, 16#7c#, 16#f1#,  --  470
      16#b9#, 16#a3#, 16#82#, 16#93#, 16#61#, 16#ba#, 16#a2#, 16#da#,
      16#de#, 16#df#, 16#db#, 16#81#, 16#9a#, 16#b9#, 16#ae#, 16#f5#,  --  480
      16#60#, 16#68#, 16#70#, 16#f1#, 16#da#, 16#ba#, 16#a2#, 16#df#,
      16#d9#, 16#ba#, 16#a2#, 16#fa#, 16#b9#, 16#a3#, 16#82#, 16#92#,  --  490
      16#db#, 16#31#, 16#ba#, 16#a2#, 16#d9#, 16#ba#, 16#a2#, 16#f8#,
      16#df#, 16#85#, 16#a4#, 16#d0#, 16#c1#, 16#bb#, 16#ad#, 16#83#,  --  4A0
      16#c2#, 16#c5#, 16#c7#, 16#b8#, 16#a2#, 16#df#, 16#df#, 16#df#,
      16#ba#, 16#a0#, 16#df#, 16#df#, 16#df#, 16#d8#, 16#d8#, 16#f1#,  --  4B0
      16#b8#, 16#aa#, 16#b3#, 16#8d#, 16#b4#, 16#98#, 16#0d#, 16#35#,
--   | CFG_MOTION_BIAS  9 bytes
      16#5d#, 16#b2#, 16#b6#, 16#ba#, 16#af#, 16#8c#, 16#96#, 16#19#,  --  4C0
--           |
      16#8f#, 16#9f#, 16#a7#, 16#0e#, 16#16#, 16#1e#, 16#b4#, 16#9a#,
      16#b8#, 16#aa#, 16#87#, 16#2c#, 16#54#, 16#7c#, 16#ba#, 16#a4#,  --  4D0
      16#b0#, 16#8a#, 16#b6#, 16#91#, 16#32#, 16#56#, 16#76#, 16#b2#,
      16#84#, 16#94#, 16#a4#, 16#c8#, 16#08#, 16#cd#, 16#d8#, 16#b8#,  --  4E0
      16#b4#, 16#b0#, 16#f1#, 16#99#, 16#82#, 16#a8#, 16#2d#, 16#55#,
      16#7d#, 16#98#, 16#a8#, 16#0e#, 16#16#, 16#1e#, 16#a2#, 16#2c#,  --  4F0
      16#54#, 16#7c#, 16#92#, 16#a4#, 16#f0#, 16#2c#, 16#50#, 16#78#,
      --  bank # 5
      16#f1#, 16#84#, 16#a8#, 16#98#, 16#c4#, 16#cd#, 16#fc#, 16#d8#,
      16#0d#, 16#db#, 16#a8#, 16#fc#, 16#2d#, 16#f3#, 16#d9#, 16#ba#,
      16#a6#, 16#f8#, 16#da#, 16#ba#, 16#a6#, 16#de#, 16#d8#, 16#ba#,
      16#b2#, 16#b6#, 16#86#, 16#96#, 16#a6#, 16#d0#, 16#f3#, 16#c8#,
      16#41#, 16#da#, 16#a6#, 16#c8#, 16#f8#, 16#d8#, 16#b0#, 16#b4#,
      16#b8#, 16#82#, 16#a8#, 16#92#, 16#f5#, 16#2c#, 16#54#, 16#88#,
      16#98#, 16#f1#, 16#35#, 16#d9#, 16#f4#, 16#18#, 16#d8#, 16#f1#,
      16#a2#, 16#d0#, 16#f8#, 16#f9#, 16#a8#, 16#84#, 16#d9#, 16#c7#,
      16#df#, 16#f8#, 16#f8#, 16#83#, 16#c5#, 16#da#, 16#df#, 16#69#,
      16#df#, 16#83#, 16#c1#, 16#d8#, 16#f4#, 16#01#, 16#14#, 16#f1#,
      16#a8#, 16#82#, 16#4e#, 16#a8#, 16#84#, 16#f3#, 16#11#, 16#d1#,
      16#82#, 16#f5#, 16#d9#, 16#92#, 16#28#, 16#97#, 16#88#, 16#f1#,
      16#09#, 16#f4#, 16#1c#, 16#1c#, 16#d8#, 16#84#, 16#a8#, 16#f3#,
      16#c0#, 16#f9#, 16#d1#, 16#d9#, 16#97#, 16#82#, 16#f1#, 16#29#,
      16#f4#, 16#0d#, 16#d8#, 16#f3#, 16#f9#, 16#f9#, 16#d1#, 16#d9#,
      16#82#, 16#f4#, 16#c2#, 16#03#, 16#d8#, 16#de#, 16#df#, 16#1a#,
      16#d8#, 16#f1#, 16#a2#, 16#fa#, 16#f9#, 16#a8#, 16#84#, 16#98#,
      16#d9#, 16#c7#, 16#df#, 16#f8#, 16#f8#, 16#f8#, 16#83#, 16#c7#,
      16#da#, 16#df#, 16#69#, 16#df#, 16#f8#, 16#83#, 16#c3#, 16#d8#,
      16#f4#, 16#01#, 16#14#, 16#f1#, 16#98#, 16#a8#, 16#82#, 16#2e#,
      16#a8#, 16#84#, 16#f3#, 16#11#, 16#d1#, 16#82#, 16#f5#, 16#d9#,
      16#92#, 16#50#, 16#97#, 16#88#, 16#f1#, 16#09#, 16#f4#, 16#1c#,
      16#d8#, 16#84#, 16#a8#, 16#f3#, 16#c0#, 16#f8#, 16#f9#, 16#d1#,
      16#d9#, 16#97#, 16#82#, 16#f1#, 16#49#, 16#f4#, 16#0d#, 16#d8#,
      16#f3#, 16#f9#, 16#f9#, 16#d1#, 16#d9#, 16#82#, 16#f4#, 16#c4#,
      16#03#, 16#d8#, 16#de#, 16#df#, 16#d8#, 16#f1#, 16#ad#, 16#88#,
      16#98#, 16#cc#, 16#a8#, 16#09#, 16#f9#, 16#d9#, 16#82#, 16#92#,
      16#a8#, 16#f5#, 16#7c#, 16#f1#, 16#88#, 16#3a#, 16#cf#, 16#94#,
      16#4a#, 16#6e#, 16#98#, 16#db#, 16#69#, 16#31#, 16#da#, 16#ad#,
      16#f2#, 16#de#, 16#f9#, 16#d8#, 16#87#, 16#95#, 16#a8#, 16#f2#,
      16#21#, 16#d1#, 16#da#, 16#a5#, 16#f9#, 16#f4#, 16#17#, 16#d9#,
      16#f1#, 16#ae#, 16#8e#, 16#d0#, 16#c0#, 16#c3#, 16#ae#, 16#82#,
      --  bank # 6
      16#c6#, 16#84#, 16#c3#, 16#a8#, 16#85#, 16#95#, 16#c8#, 16#a5#,
      16#88#, 16#f2#, 16#c0#, 16#f1#, 16#f4#, 16#01#, 16#0e#, 16#f1#,
      16#8e#, 16#9e#, 16#a8#, 16#c6#, 16#3e#, 16#56#, 16#f5#, 16#54#,
      16#f1#, 16#88#, 16#72#, 16#f4#, 16#01#, 16#15#, 16#f1#, 16#98#,
      16#45#, 16#85#, 16#6e#, 16#f5#, 16#8e#, 16#9e#, 16#04#, 16#88#,
      16#f1#, 16#42#, 16#98#, 16#5a#, 16#8e#, 16#9e#, 16#06#, 16#88#,
      16#69#, 16#f4#, 16#01#, 16#1c#, 16#f1#, 16#98#, 16#1e#, 16#11#,
      16#08#, 16#d0#, 16#f5#, 16#04#, 16#f1#, 16#1e#, 16#97#, 16#02#,
      16#02#, 16#98#, 16#36#, 16#25#, 16#db#, 16#f9#, 16#d9#, 16#85#,
      16#a5#, 16#f3#, 16#c1#, 16#da#, 16#85#, 16#a5#, 16#f3#, 16#df#,
      16#d8#, 16#85#, 16#95#, 16#a8#, 16#f3#, 16#09#, 16#da#, 16#a5#,
      16#fa#, 16#d8#, 16#82#, 16#92#, 16#a8#, 16#f5#, 16#78#, 16#f1#,
      16#88#, 16#1a#, 16#84#, 16#9f#, 16#26#, 16#88#, 16#98#, 16#21#,
      16#da#, 16#f4#, 16#1d#, 16#f3#, 16#d8#, 16#87#, 16#9f#, 16#39#,
      16#d1#, 16#af#, 16#d9#, 16#df#, 16#df#, 16#fb#, 16#f9#, 16#f4#,
      16#0c#, 16#f3#, 16#d8#, 16#fa#, 16#d0#, 16#f8#, 16#da#, 16#f9#,
      16#f9#, 16#d0#, 16#df#, 16#d9#, 16#f9#, 16#d8#, 16#f4#, 16#0b#,
      16#d8#, 16#f3#, 16#87#, 16#9f#, 16#39#, 16#d1#, 16#af#, 16#d9#,
      16#df#, 16#df#, 16#f4#, 16#1d#, 16#f3#, 16#d8#, 16#fa#, 16#fc#,
      16#a8#, 16#69#, 16#f9#, 16#f9#, 16#af#, 16#d0#, 16#da#, 16#de#,
      16#fa#, 16#d9#, 16#f8#, 16#8f#, 16#9f#, 16#a8#, 16#f1#, 16#cc#,
      16#f3#, 16#98#, 16#db#, 16#45#, 16#d9#, 16#af#, 16#df#, 16#d0#,
      16#f8#, 16#d8#, 16#f1#, 16#8f#, 16#9f#, 16#a8#, 16#ca#, 16#f3#,
      16#88#, 16#09#, 16#da#, 16#af#, 16#8f#, 16#cb#, 16#f8#, 16#d8#,
      16#f2#, 16#ad#, 16#97#, 16#8d#, 16#0c#, 16#d9#, 16#a5#, 16#df#,
      16#f9#, 16#ba#, 16#a6#, 16#f3#, 16#fa#, 16#f4#, 16#12#, 16#f2#,
      16#d8#, 16#95#, 16#0d#, 16#d1#, 16#d9#, 16#ba#, 16#a6#, 16#f3#,
      16#fa#, 16#da#, 16#a5#, 16#f2#, 16#c1#, 16#ba#, 16#a6#, 16#f3#,
      16#df#, 16#d8#, 16#f1#, 16#ba#, 16#b2#, 16#b6#, 16#86#, 16#96#,
      16#a6#, 16#d0#, 16#ca#, 16#f3#, 16#49#, 16#da#, 16#a6#, 16#cb#,
      16#f8#, 16#d8#, 16#b0#, 16#b4#, 16#b8#, 16#d8#, 16#ad#, 16#84#,
      16#f2#, 16#c0#, 16#df#, 16#f1#, 16#8f#, 16#cb#, 16#c3#, 16#a8#,
      --  bank # 7 */
      16#b2#, 16#b6#, 16#86#, 16#96#, 16#c8#, 16#c1#, 16#cb#, 16#c3#,
      16#f3#, 16#b0#, 16#b4#, 16#88#, 16#98#, 16#a8#, 16#21#, 16#db#,
      16#71#, 16#8d#, 16#9d#, 16#71#, 16#85#, 16#95#, 16#21#, 16#d9#,
      16#ad#, 16#f2#, 16#fa#, 16#d8#, 16#85#, 16#97#, 16#a8#, 16#28#,
      16#d9#, 16#f4#, 16#08#, 16#d8#, 16#f2#, 16#8d#, 16#29#, 16#da#,
      16#f4#, 16#05#, 16#d9#, 16#f2#, 16#85#, 16#a4#, 16#c2#, 16#f2#,
      16#d8#, 16#a8#, 16#8d#, 16#94#, 16#01#, 16#d1#, 16#d9#, 16#f4#,
      16#11#, 16#f2#, 16#d8#, 16#87#, 16#21#, 16#d8#, 16#f4#, 16#0a#,
      16#d8#, 16#f2#, 16#84#, 16#98#, 16#a8#, 16#c8#, 16#01#, 16#d1#,
      16#d9#, 16#f4#, 16#11#, 16#d8#, 16#f3#, 16#a4#, 16#c8#, 16#bb#,
      16#af#, 16#d0#, 16#f2#, 16#de#, 16#f8#, 16#f8#, 16#f8#, 16#f8#,
      16#f8#, 16#f8#, 16#f8#, 16#f8#, 16#d8#, 16#f1#, 16#b8#, 16#f6#,
      16#b5#, 16#b9#, 16#b0#, 16#8a#, 16#95#, 16#a3#, 16#de#, 16#3c#,
      16#a3#, 16#d9#, 16#f8#, 16#d8#, 16#5c#, 16#a3#, 16#d9#, 16#f8#,
      16#d8#, 16#7c#, 16#a3#, 16#d9#, 16#f8#, 16#d8#, 16#f8#, 16#f9#,
      16#d1#, 16#a5#, 16#d9#, 16#df#, 16#da#, 16#fa#, 16#d8#, 16#b1#,
      16#85#, 16#30#, 16#f7#, 16#d9#, 16#de#, 16#d8#, 16#f8#, 16#30#,
      16#ad#, 16#da#, 16#de#, 16#d8#, 16#f2#, 16#b4#, 16#8c#, 16#99#,
      16#a3#, 16#2d#, 16#55#, 16#7d#, 16#a0#, 16#83#, 16#df#, 16#df#,
      16#df#, 16#b5#, 16#91#, 16#a0#, 16#f6#, 16#29#, 16#d9#, 16#fb#,
      16#d8#, 16#a0#, 16#fc#, 16#29#, 16#d9#, 16#fa#, 16#d8#, 16#a0#,
      16#d0#, 16#51#, 16#d9#, 16#f8#, 16#d8#, 16#fc#, 16#51#, 16#d9#,
      16#f9#, 16#d8#, 16#79#, 16#d9#, 16#fb#, 16#d8#, 16#a0#, 16#d0#,
      16#fc#, 16#79#, 16#d9#, 16#fa#, 16#d8#, 16#a1#, 16#f9#, 16#f9#,
      16#f9#, 16#f9#, 16#f9#, 16#a0#, 16#da#, 16#df#, 16#df#, 16#df#,
      16#d8#, 16#a1#, 16#f8#, 16#f8#, 16#f8#, 16#f8#, 16#f8#, 16#ac#,
      16#de#, 16#f8#, 16#ad#, 16#de#, 16#83#, 16#93#, 16#ac#, 16#2c#,
      16#54#, 16#7c#, 16#f1#, 16#a8#, 16#df#, 16#df#, 16#df#, 16#f6#,
      16#9d#, 16#2c#, 16#da#, 16#a0#, 16#df#, 16#d9#, 16#fa#, 16#db#,
      16#2d#, 16#f8#, 16#d8#, 16#a8#, 16#50#, 16#da#, 16#a0#, 16#d0#,
      16#de#, 16#d9#, 16#d0#, 16#f8#, 16#f8#, 16#f8#, 16#db#, 16#55#,
      16#f8#, 16#d8#, 16#a8#, 16#78#, 16#da#, 16#a0#, 16#d0#, 16#df#,
      --  bank # 8
      16#d9#, 16#d0#, 16#fa#, 16#f8#, 16#f8#, 16#f8#, 16#f8#, 16#db#,
      16#7d#, 16#f8#, 16#d8#, 16#9c#, 16#a8#, 16#8c#, 16#f5#, 16#30#,
      16#db#, 16#38#, 16#d9#, 16#d0#, 16#de#, 16#df#, 16#a0#, 16#d0#,
      16#de#, 16#df#, 16#d8#, 16#a8#, 16#48#, 16#db#, 16#58#, 16#d9#,
      16#df#, 16#d0#, 16#de#, 16#a0#, 16#df#, 16#d0#, 16#de#, 16#d8#,
      16#a8#, 16#68#, 16#db#, 16#70#, 16#d9#, 16#df#, 16#df#, 16#a0#,
      16#df#, 16#df#, 16#d8#, 16#f1#, 16#a8#, 16#88#, 16#90#, 16#2c#,
      16#54#, 16#7c#, 16#98#, 16#a8#, 16#d0#, 16#5c#, 16#38#, 16#d1#,
      16#da#, 16#f2#, 16#ae#, 16#8c#, 16#df#, 16#f9#, 16#d8#, 16#b0#,
      16#87#, 16#a8#, 16#c1#, 16#c1#, 16#b1#, 16#88#, 16#a8#, 16#c6#,
      16#f9#, 16#f9#, 16#da#, 16#36#, 16#d8#, 16#a8#, 16#f9#, 16#da#,
      16#36#, 16#d8#, 16#a8#, 16#f9#, 16#da#, 16#36#, 16#d8#, 16#a8#,
      16#f9#, 16#da#, 16#36#, 16#d8#, 16#a8#, 16#f9#, 16#da#, 16#36#,
      16#d8#, 16#f7#, 16#8d#, 16#9d#, 16#ad#, 16#f8#, 16#18#, 16#da#,
      16#f2#, 16#ae#, 16#df#, 16#d8#, 16#f7#, 16#ad#, 16#fa#, 16#30#,
      16#d9#, 16#a4#, 16#de#, 16#f9#, 16#d8#, 16#f2#, 16#ae#, 16#de#,
      16#fa#, 16#f9#, 16#83#, 16#a7#, 16#d9#, 16#c3#, 16#c5#, 16#c7#,
      16#f1#, 16#88#, 16#9b#, 16#a7#, 16#7a#, 16#ad#, 16#f7#, 16#de#,
      16#df#, 16#a4#, 16#f8#, 16#84#, 16#94#, 16#08#, 16#a7#, 16#97#,
      16#f3#, 16#00#, 16#ae#, 16#f2#, 16#98#, 16#19#, 16#a4#, 16#88#,
      16#c6#, 16#a3#, 16#94#, 16#88#, 16#f6#, 16#32#, 16#df#, 16#f2#,
      16#83#, 16#93#, 16#db#, 16#09#, 16#d9#, 16#f2#, 16#aa#, 16#df#,
      16#d8#, 16#d8#, 16#ae#, 16#f8#, 16#f9#, 16#d1#, 16#da#, 16#f3#,
      16#a4#, 16#de#, 16#a7#, 16#f1#, 16#88#, 16#9b#, 16#7a#, 16#d8#,
      16#f3#, 16#84#, 16#94#, 16#ae#, 16#19#, 16#f9#, 16#da#, 16#aa#,
      16#f1#, 16#df#, 16#d8#, 16#a8#, 16#81#, 16#c0#, 16#c3#, 16#c5#,
      16#c7#, 16#a3#, 16#92#, 16#83#, 16#f6#, 16#28#, 16#ad#, 16#de#,
      16#d9#, 16#f8#, 16#d8#, 16#a3#, 16#50#, 16#ad#, 16#d9#, 16#f8#,
      16#d8#, 16#a3#, 16#78#, 16#ad#, 16#d9#, 16#f8#, 16#d8#, 16#f8#,
      16#f9#, 16#d1#, 16#a1#, 16#da#, 16#de#, 16#c3#, 16#c5#, 16#c7#,
      16#d8#, 16#a1#, 16#81#, 16#94#, 16#f8#, 16#18#, 16#f2#, 16#b0#,
      16#89#, 16#ac#, 16#c3#, 16#c5#, 16#c7#, 16#f1#, 16#d8#, 16#b8#,
      --  bank # 9
      16#b4#, 16#b0#, 16#97#, 16#86#, 16#a8#, 16#31#, 16#9b#, 16#06#,
      16#99#, 16#07#, 16#ab#, 16#97#, 16#28#, 16#88#, 16#9b#, 16#f0#,
      16#0c#, 16#20#, 16#14#, 16#40#, 16#b0#, 16#b4#, 16#b8#, 16#f0#,
      16#a8#, 16#8a#, 16#9a#, 16#28#, 16#50#, 16#78#, 16#b7#, 16#9b#,
      16#a8#, 16#29#, 16#51#, 16#79#, 16#24#, 16#70#, 16#59#, 16#44#,
      16#69#, 16#38#, 16#64#, 16#48#, 16#31#, 16#f1#, 16#bb#, 16#ab#,
      16#88#, 16#00#, 16#2c#, 16#54#, 16#7c#, 16#f0#, 16#b3#, 16#8b#,
      16#b8#, 16#a8#, 16#04#, 16#28#, 16#50#, 16#78#, 16#f1#, 16#b0#,
      16#88#, 16#b4#, 16#97#, 16#26#, 16#a8#, 16#59#, 16#98#, 16#bb#,
      16#ab#, 16#b3#, 16#8b#, 16#02#, 16#26#, 16#46#, 16#66#, 16#b0#,
      16#b8#, 16#f0#, 16#8a#, 16#9c#, 16#a8#, 16#29#, 16#51#, 16#79#,
      16#8b#, 16#29#, 16#51#, 16#79#, 16#8a#, 16#24#, 16#70#, 16#59#,
      16#8b#, 16#20#, 16#58#, 16#71#, 16#8a#, 16#44#, 16#69#, 16#38#,
      16#8b#, 16#39#, 16#40#, 16#68#, 16#8a#, 16#64#, 16#48#, 16#31#,
      16#8b#, 16#30#, 16#49#, 16#60#, 16#88#, 16#f1#, 16#ac#, 16#00#,
      16#2c#, 16#54#, 16#7c#, 16#f0#, 16#8c#, 16#a8#, 16#04#, 16#28#,
      16#50#, 16#78#, 16#f1#, 16#88#, 16#97#, 16#26#, 16#a8#, 16#59#,
      16#98#, 16#ac#, 16#8c#, 16#02#, 16#26#, 16#46#, 16#66#, 16#f0#,
      16#89#, 16#9c#, 16#a8#, 16#29#, 16#51#, 16#79#, 16#24#, 16#70#,
      16#59#, 16#44#, 16#69#, 16#38#, 16#64#, 16#48#, 16#31#, 16#a9#,
      16#88#, 16#09#, 16#20#, 16#59#, 16#70#, 16#ab#, 16#11#, 16#38#,
      16#40#, 16#69#, 16#a8#, 16#19#, 16#31#, 16#48#, 16#60#, 16#8c#,
      16#a8#, 16#3c#, 16#41#, 16#5c#, 16#20#, 16#7c#, 16#00#, 16#f1#,
      16#87#, 16#98#, 16#19#, 16#86#, 16#a8#, 16#6e#, 16#76#, 16#7e#,
      16#a9#, 16#99#, 16#88#, 16#2d#, 16#55#, 16#7d#, 16#d8#, 16#b1#,
      16#b5#, 16#b9#, 16#a3#, 16#df#, 16#df#, 16#df#, 16#ae#, 16#d0#,
      16#df#, 16#aa#, 16#d0#, 16#de#, 16#f2#, 16#ab#, 16#f8#, 16#f9#,
      16#d9#, 16#b0#, 16#87#, 16#c4#, 16#aa#, 16#f1#, 16#df#, 16#df#,
      16#bb#, 16#af#, 16#df#, 16#df#, 16#b9#, 16#d8#, 16#b1#, 16#f1#,
      16#a3#, 16#97#, 16#8e#, 16#60#, 16#df#, 16#b0#, 16#84#, 16#f2#,
      16#c8#, 16#f8#, 16#f9#, 16#d9#, 16#de#, 16#d8#, 16#93#, 16#85#,
      16#f1#, 16#4a#, 16#b1#, 16#83#, 16#a3#, 16#08#, 16#b5#, 16#83#,
    --   bank # 10
      16#9a#, 16#08#, 16#10#, 16#b7#, 16#9f#, 16#10#, 16#d8#, 16#f1#,  --  A00
      16#b0#, 16#ba#, 16#ae#, 16#b0#, 16#8a#, 16#c2#, 16#b2#, 16#b6#,
      16#8e#, 16#9e#, 16#f1#, 16#fb#, 16#d9#, 16#f4#, 16#1d#, 16#d8#,  --  A10
      16#f9#, 16#d9#, 16#0c#, 16#f1#, 16#d8#, 16#f8#, 16#f8#, 16#ad#,
      16#61#, 16#d9#, 16#ae#, 16#fb#, 16#d8#, 16#f4#, 16#0c#, 16#f1#,  --  A20
      16#d8#, 16#f8#, 16#f8#, 16#ad#, 16#19#, 16#d9#, 16#ae#, 16#fb#,
      16#df#, 16#d8#, 16#f4#, 16#16#, 16#f1#, 16#d8#, 16#f8#, 16#ad#,  --  A30
      16#8d#, 16#61#, 16#d9#, 16#f4#, 16#f4#, 16#ac#, 16#f5#, 16#9c#,
      16#9c#, 16#8d#, 16#df#, 16#2b#, 16#ba#, 16#b6#, 16#ae#, 16#fa#,  --  A40
      16#f8#, 16#f4#, 16#0b#, 16#d8#, 16#f1#, 16#ae#, 16#d0#, 16#f8#,
      16#ad#, 16#51#, 16#da#, 16#ae#, 16#fa#, 16#f8#, 16#f1#, 16#d8#,  --  A50
      16#b9#, 16#b1#, 16#b6#, 16#a3#, 16#83#, 16#9c#, 16#08#, 16#b9#,
      16#b1#, 16#83#, 16#9a#, 16#b5#, 16#aa#, 16#c0#, 16#fd#, 16#30#,  --  A60
      16#83#, 16#b7#, 16#9f#, 16#10#, 16#b5#, 16#8b#, 16#93#, 16#f2#,
      16#02#, 16#02#, 16#d1#, 16#ab#, 16#da#, 16#de#, 16#d8#, 16#f1#,  --  A70
      16#b0#, 16#80#, 16#ba#, 16#ab#, 16#c0#, 16#c3#, 16#b2#, 16#84#,
      16#c1#, 16#c3#, 16#d8#, 16#b1#, 16#b9#, 16#f3#, 16#8b#, 16#a3#,  --  A80
--                   | CFG_FIFO_ON_EVENT  11 bytes
      16#91#, 16#b6#, 16#09#, 16#b4#, 16#d9#, 16#ab#, 16#de#, 16#b0#,
--                                           |
      16#87#, 16#9c#, 16#b9#, 16#a3#, 16#dd#, 16#f1#, 16#b3#, 16#8b#,  --  A90
      16#8b#, 16#8b#, 16#8b#, 16#8b#, 16#b0#, 16#87#, 16#a3#, 16#a3#,
--   | CFG_LP_QUAT/3A_LPQ_EN 4 bytes |------  ------ | CFG_8/6A_LPQ_EN
      16#a3#, 16#a3#, 16#b2#, 16#8b#, 16#b6#, 16#9b#, 16#f2#, 16#a3#,  --  AA0
--    4 bytes         | CFG_GYRO_RAW_DATA  4 bytes    |       | RAW_DATA_EN
      16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#,
--    10 bytes
      16#a3#, 16#f1#, 16#b0#, 16#87#, 16#b5#, 16#9a#, 16#a3#, 16#f3#,  --  AB0
--           |
      16#9b#, 16#a3#, 16#a3#, 16#dc#, 16#ba#, 16#ac#, 16#df#, 16#b9#,
      16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#,  --  AC0
--           | FIFO_RATE_DIV_EN  12 bytes
      16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#, 16#a3#,
--                                           |
      16#d8#, 16#d8#, 16#d8#, 16#bb#, 16#b3#, 16#b7#, 16#f1#, 16#aa#,  --  AD0
      16#f9#, 16#da#, 16#ff#, 16#d9#, 16#80#, 16#9a#, 16#aa#, 16#28#,
      16#b4#, 16#80#, 16#98#, 16#a7#, 16#20#, 16#b7#, 16#97#, 16#87#,  --  AE0
      16#a8#, 16#66#, 16#88#, 16#f0#, 16#79#, 16#51#, 16#f1#, 16#90#,
      16#2c#, 16#87#, 16#0c#, 16#a7#, 16#81#, 16#97#, 16#62#, 16#93#,  --  AF0
      16#f0#, 16#71#, 16#71#, 16#60#, 16#85#, 16#94#, 16#01#, 16#29#,
      --   bank # 11
      16#51#, 16#79#, 16#90#, 16#a5#, 16#f1#, 16#28#, 16#4c#, 16#6c#,
      16#87#, 16#0c#, 16#95#, 16#18#, 16#85#, 16#78#, 16#a3#, 16#83#,
      16#90#, 16#28#, 16#4c#, 16#6c#, 16#88#, 16#6c#, 16#d8#, 16#f3#,
      16#a2#, 16#82#, 16#00#, 16#f2#, 16#10#, 16#a8#, 16#92#, 16#19#,
      16#80#, 16#a2#, 16#f2#, 16#d9#, 16#26#, 16#d8#, 16#f1#, 16#88#,
      16#a8#, 16#4d#, 16#d9#, 16#48#, 16#d8#, 16#96#, 16#a8#, 16#39#,
      16#80#, 16#d9#, 16#3c#, 16#d8#, 16#95#, 16#80#, 16#a8#, 16#39#,
      16#a6#, 16#86#, 16#98#, 16#d9#, 16#2c#, 16#da#, 16#87#, 16#a7#,
      16#2c#, 16#d8#, 16#a8#, 16#89#, 16#95#, 16#19#, 16#a9#, 16#80#,
      16#d9#, 16#38#, 16#d8#, 16#a8#, 16#89#, 16#39#, 16#a9#, 16#80#,
      16#da#, 16#3c#, 16#d8#, 16#a8#, 16#2e#, 16#a8#, 16#39#, 16#90#,
      16#d9#, 16#0c#, 16#d8#, 16#a8#, 16#95#, 16#31#, 16#98#, 16#d9#,
      16#0c#, 16#d8#, 16#a8#, 16#09#, 16#d9#, 16#ff#, 16#d8#, 16#01#,
      16#da#, 16#ff#, 16#d8#, 16#95#, 16#39#, 16#a9#, 16#da#, 16#26#,
      16#ff#, 16#d8#, 16#90#, 16#a8#, 16#0d#, 16#89#, 16#99#, 16#a8#,
      16#10#, 16#80#, 16#98#, 16#21#, 16#da#, 16#2e#, 16#d8#, 16#89#,
      16#99#, 16#a8#, 16#31#, 16#80#, 16#da#, 16#2e#, 16#d8#, 16#a8#,
      16#86#, 16#96#, 16#31#, 16#80#, 16#da#, 16#2e#, 16#d8#, 16#a8#,
      16#87#, 16#31#, 16#80#, 16#da#, 16#2e#, 16#d8#, 16#a8#, 16#82#,
      16#92#, 16#f3#, 16#41#, 16#80#, 16#f1#, 16#d9#, 16#2e#, 16#d8#,
      16#a8#, 16#82#, 16#f3#, 16#19#, 16#80#, 16#f1#, 16#d9#, 16#2e#,
      16#d8#, 16#82#, 16#ac#, 16#f3#, 16#c0#, 16#a2#, 16#80#, 16#22#,
      16#f1#, 16#a6#, 16#2e#, 16#a7#, 16#2e#, 16#a9#, 16#22#, 16#98#,
      16#a8#, 16#29#, 16#da#, 16#ac#, 16#de#, 16#ff#, 16#d8#, 16#a2#,
      16#f2#, 16#2a#, 16#f1#, 16#a9#, 16#2e#, 16#82#, 16#92#, 16#a8#,
      16#f2#, 16#31#, 16#80#, 16#a6#, 16#96#, 16#f1#, 16#d9#, 16#00#,
      16#ac#, 16#8c#, 16#9c#, 16#0c#, 16#30#, 16#ac#, 16#de#, 16#d0#,
      16#de#, 16#ff#, 16#d8#, 16#8c#, 16#9c#, 16#ac#, 16#d0#, 16#10#,
      16#ac#, 16#de#, 16#80#, 16#92#, 16#a2#, 16#f2#, 16#4c#, 16#82#,
      16#a8#, 16#f1#, 16#ca#, 16#f2#, 16#35#, 16#f1#, 16#96#, 16#88#,
      16#a6#, 16#d9#, 16#00#, 16#d8#, 16#f1#, 16#ff#);

--     Firmware_Start_Address : constant := 16#0400#;

   DINA20 : constant := 16#20#;
   DINA28 : constant := 16#28#;
   DINA30 : constant := 16#30#;
   DINA38 : constant := 16#38#;

   DINAF2 : constant := 16#F2#;
   DINAAB : constant := 16#AB#;
   DINAAA : constant := 16#AA#;
   DINAF1 : constant := 16#F1#;
   DINADF : constant := 16#DF#;

   DINA80 : constant := 16#80#;
   DINA90 : constant := 16#90#;
   DINAFE : constant := 16#FE#;

   DINAC0 : constant := 16#b0#;  --  ???
   DINAC2 : constant := 16#b4#;  --  ???
   DINBC0 : constant := 16#c0#;
   DINBC2 : constant := 16#c2#;
   DINBC4 : constant := 16#c4#;
   DINBC6 : constant := 16#c6#;

   package Registers is

      --  GYRO_SF (104..197/68..6B)

      type GYRO_SF_Register is record
         GYRO_SF : Interfaces.Unsigned_32;
      end record
        with Pack,
             Object_Size          => 32,
             Bit_Order            => System.High_Order_First,
             Scalar_Storage_Order => System.High_Order_First;

      --  FIFO_RATE_DIV (534..535/216..217)

      type FIFO_RATE_DIV_Register is record
         FIFO_RATE_DIV : Interfaces.Integer_16;
      end record
        with Pack,
             Object_Size          => 16,
             Bit_Order            => System.High_Order_First,
             Scalar_Storage_Order => System.High_Order_First;

   end Registers;

   GYRO_SF : constant := 46_850_825 * 200 / DMP_Sample_Rate;

   D_0_104_Address           : constant := 104;      --   68
   D_0_104_Length            : constant := 4;

   D_0_22_Address            : constant := 22 + 512;  --  216 | 534 = FIFO_RATE_DIV
--     D_0_22_Length             : constant := 2;

   CFG_MOTION_BIAS_Address   : constant := 1208;  --  4B8
   CFG_MOTION_BIAS_Length    : constant := 9;

   CFG_FIFO_ON_EVENT_Address : constant := 2690;  --  A82
   CFG_FIFO_ON_EVENT_Length  : constant := 11;

   CFG_LP_QUAT_Address       : constant := 2712;  --  A98 | 3A_LPQ_EN = A9D
   CFG_LP_QUAT_Length        : constant := 4;

   CFG_8_Address             : constant := 2718;  --  A9E | 6A_LPQ_EN = AA3
   CFG_8_Length              : constant := 4;

   CFG_GYRO_RAW_DATA_Address : constant := 2722;  --  AA2
   CFG_GYRO_RAW_DATA_Length  : constant := 4;

   CFG_15_Address            : constant := 2727;  --  AA7 | RAW_DATA_EN = 2731
   CFG_15_Length             : constant := 10;

   CFG_6_Address             : constant := 2753;  --  AC1 | FIFO_RATE_DIV_EN = 2756
   CFG_6_Length              : constant := 12;

   Firmware_Buffer : A0B.I2C.Unsigned_8_Array (0 .. Firmware_Size - 1);

   procedure Write_DMP_Memory
     (Address : Interfaces.Unsigned_32;
      Data    : A0B.I2C.Unsigned_8_Array);

   ---------------------
   -- Enable_Features --
   ---------------------

   procedure Enable_Features
     (Self                  : in out Abstract_MPU_Sensor'Class;
      Accelerometer         : Accelerometer_Data_Mode;
      Gyroscope             : Gyroscope_Data_Mode;
      Quaternion            : Quaternion_Mode;
      Gyroscope_Calibration : Boolean;
      Tap                   : Boolean;
      Android_Orientation   : Boolean)
   is
      use type Interfaces.Unsigned_16;

   begin
      Self.FIFO_Packet_Size := 0;

      --  Set integration scale factor.

      declare
         D_0_104   : Registers.GYRO_SF_Register :=
           (GYRO_SF => GYRO_SF);
         D_0_104_B : A0B.I2C.Unsigned_8_Array (1 .. D_0_104_Length)
           with Import, Address => D_0_104'Address;

      begin
         Write_DMP_Memory (D_0_104_Address, D_0_104_B);
         --  XXX This value is already set in firmware binary.
      end;

      --  Send sensor data to the FIFO.

      declare
         RAW_DATA_EN : A0B.I2C.Unsigned_8_Array (1 .. CFG_15_Length) :=
           (16#A3#, 16#A3#, 16#A3#, 16#A3#, 16#A3#, 16#A3#, 16#A3#, 16#A3#,
            16#A3#, 16#A3#);

      begin
         if Accelerometer /= None then
            RAW_DATA_EN (2 .. 4) := (16#C0#, 16#C8#, 16#C2#);
            Self.DMP_Accelerometer_Enabled := True;
            Self.FIFO_Packet_Size := @ + ACCEL_OUT_Length;

         else
            Self.DMP_Accelerometer_Enabled := False;
         end if;

         if Gyroscope /= None then
            RAW_DATA_EN (5 .. 7) := (16#C4#, 16#CC#, 16#C6#);
            Self.DMP_Gyroscope_Enabled := True;
            Self.FIFO_Packet_Size := @ + GYRO_OUT_Length;

         else
            Self.DMP_Gyroscope_Enabled := False;
         end if;

         Write_DMP_Memory (CFG_15_Address, RAW_DATA_EN);
      end;

      if Gyroscope /= None then
         declare
            CFG_GYRO_RAW_DATA : A0B.I2C.Unsigned_8_Array
                                  (1 .. CFG_GYRO_RAW_DATA_Length);

         begin
            if Gyroscope = Calibrated then
               CFG_GYRO_RAW_DATA := (16#B2#, 16#8B#, 16#B6#, 16#9B#);

            else
               CFG_GYRO_RAW_DATA := (DINAC0, DINA80, DINAC2, DINA90);
            end if;

            Write_DMP_Memory (CFG_GYRO_RAW_DATA_Address, CFG_GYRO_RAW_DATA);
         end;
      end if;

      declare
         CFG_MOTION_BIAS : A0B.I2C.Unsigned_8_Array
                             (1 .. CFG_MOTION_BIAS_Length);

      begin
         if Gyroscope_Calibration then
            CFG_MOTION_BIAS :=
              (16#b8#, 16#aa#, 16#b3#, 16#8d#, 16#b4#, 16#98#, 16#0d#, 16#35#,
               16#5d#);

         else
            CFG_MOTION_BIAS :=
              (16#b8#, 16#aa#, 16#aa#, 16#aa#, 16#b0#, 16#88#, 16#c3#, 16#c5#,
               16#c7#);
         end if;

         Write_DMP_Memory (CFG_MOTION_BIAS_Address, CFG_MOTION_BIAS);
      end;

      --  Quaternion

      Self.DMP_Quaternion_Enabled := False;

      declare
         CFG_LP_QUAT : A0B.I2C.Unsigned_8_Array (1 .. CFG_LP_QUAT_Length);

      begin
         if Quaternion = Quaternion_3 then
            CFG_LP_QUAT := (DINBC0, DINBC2, DINBC4, DINBC6);
            Self.FIFO_Packet_Size := @ + 16;
            Self.DMP_Quaternion_Enabled := True;

         else
            CFG_LP_QUAT := (16#8B#, 16#8B#, 16#8B#, 16#8B#);
         end if;

         Write_DMP_Memory (CFG_LP_QUAT_Address, CFG_LP_QUAT);
      end;

      declare
         CFG_8 : A0B.I2C.Unsigned_8_Array (1 .. CFG_8_Length);

      begin
         if Quaternion = Quaternion_6 then
            CFG_8 := (DINA20, DINA28, DINA30, DINA38);
            Self.FIFO_Packet_Size := @ + 16;
            Self.DMP_Quaternion_Enabled := True;

         else
            CFG_8 := (16#A3#, 16#A3#, 16#A3#, 16#A3#);
         end if;

         Write_DMP_Memory (CFG_8_Address, CFG_8);
      end;

      --  XXX Gestures is not supported.
   end Enable_Features;

   ----------------
   -- Initialize --
   ----------------

   procedure Initialize is
   begin
      Firmware_Buffer := Firmware_Binary;
   end Initialize;

   -------------------
   -- Set_FIFO_Rate --
   -------------------

   procedure Set_FIFO_Rate (FIFO_Rate : FIFO_Rate_Type) is
      use type Interfaces.Integer_16;

      D_0_22   : constant Registers.FIFO_RATE_DIV_Register :=
        (FIFO_RATE_DIV =>
           (DMP_Sample_Rate / Interfaces.Integer_16 (FIFO_Rate) - 1));
      D_0_22_B : constant A0B.I2C.Unsigned_8_Array (1 .. 2)
        with Import, Address => D_0_22'Address;
      CFG_6    : constant A0B.I2C.Unsigned_8_Array (1 .. CFG_6_Length) :=
        (DINAFE, DINAF2, DINAAB, 16#C4#, DINAAA, DINAF1, DINADF, DINADF,
         16#BB#, 16#AF#, DINADF, DINADF);

   begin
      Write_DMP_Memory (D_0_22_Address, D_0_22_B);
      Write_DMP_Memory (CFG_6_Address, CFG_6);
   end Set_FIFO_Rate;

   ------------------------
   -- Set_Interrupt_Mode --
   ------------------------

   procedure Set_Interrupt_Mode (Mode : Interrupt_Mode) is
      B : A0B.I2C.Unsigned_8_Array (1 .. CFG_FIFO_ON_EVENT_Length);

   begin
      case Mode is
         when Continuous =>
            B := (16#D8#, 16#B1#, 16#B9#, 16#F3#,
                  16#8b#, 16#a3#, 16#91#, 16#b6#,
                  16#09#, 16#b4#, 16#d9#);

         when Gesture =>
            B := (16#da#, 16#b1#, 16#b9#, 16#f3#,
                  16#8b#, 16#a3#, 16#91#, 16#b6#,
                  16#da#, 16#b4#, 16#da#);
      end case;

      Write_DMP_Memory (CFG_FIFO_ON_EVENT_Address, B);
   end Set_Interrupt_Mode;

--     ------------------------
--     -- Unpack_FIFO_Packet --
--     ------------------------
--
--     procedure Unpack_FIFO_Packet (Self : in out Abstract_MPU_Sensor'Class) is
--
--        use type System.Storage_Elements.Storage_Offset;
--
--        Offset : System.Storage_Elements.Storage_Offset := 0;
--        Data   : Raw_Data renames Self.Raw_Data (not Self.User_Bank);
--
--     begin
--        if Self.DMP_Quaternion_Enabled then
--           --  XXX GNAT FSF 12.2 bug: halt on "Data.QUAT := Aux" assignment. So,
--           --  rewrite to copy component by component.
--
--           declare
--              Aux : constant MPU.Registers.DMP_QUAT_OUT_Register
--                with Import, Address => Self.Buffer (1)'Address + Offset;
--
--           begin
--              Data.QUAT.Q0 := Aux.Q0;
--              Data.QUAT.Q1 := Aux.Q1;
--              Data.QUAT.Q2 := Aux.Q2;
--              Data.QUAT.Q3 := Aux.Q3;
--              Offset       := @ + DMP_QUAT_OUT_Length;
--           end;
--
--        else
--           Data.QUAT.Q0 := 0;
--           Data.QUAT.Q1 := 0;
--           Data.QUAT.Q2 := 0;
--           Data.QUAT.Q3 := 0;
--        end if;
--
--        if Self.DMP_Accelerometer_Enabled then
--           declare
--              Aux : constant MPU.Registers.ACCEL_OUT_Register
--                with Import, Address => Self.Buffer'Address + Offset;
--
--           begin
--              Data.ACCEL := Aux;
--              Offset     := @ + ACCEL_OUT_Length;
--           end;
--
--        else
--           Data.ACCEL := (others => 0);
--        end if;
--
--        Data.TEMP := (others => 0);
--        --  Not available in DMP mode.
--
--        if Self.DMP_Gyroscope_Enabled then
--           declare
--              Aux : constant MPU.Registers.GYRO_OUT_Register
--                with Import, Address => Self.Buffer'Address + Offset;
--
--           begin
--              Data.GYRO := Aux;
--              Offset    := @ + GYRO_OUT_Length;
--           end;
--
--        else
--           Data.GYRO := (others => 0);
--        end if;
--
--        --  ??? Gesture into not supported
--
--        Data.Timestamp := Self.Clocks.Clock;
--        Self.User_Bank := not @;
--     end Unpack_FIFO_Packet;
--
--     ---------------------
--     -- Upload_Firmware --
--     ---------------------
--
--     procedure Upload_Firmware
--       (Self    : in out Abstract_MPU_Sensor'Class;
--        Success : in out Boolean) is
--     begin
--        Self.Upload_Firmware (Firmware_Binary, Firmware_Start_Address, Success);
--     end Upload_Firmware;

   ----------------------
   -- Write_DMP_Memory --
   ----------------------

   procedure Write_DMP_Memory
     (Address : Interfaces.Unsigned_32;
      Data    : A0B.I2C.Unsigned_8_Array) is
   begin
      Firmware_Buffer (Address .. Address + Data'Length - 1) := Data;
      --
      --  for J of Data'Range loop
      --     Firmware_Buffer (Firmware_Buffer'First + J - Data'First)
      --  end loop;
   end Write_DMP_Memory;

end A0B.MPUXXXX.DMP612;
