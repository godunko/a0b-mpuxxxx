--
--  Copyright (C) 2023-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  Digital Motion Processor (DMP) Firmware 6.12 support for
--  MPU-6000/MPU-6050/MPU-6500/MPU-9150/MPU-9250 family.

pragma Restrictions (No_Elaboration_Code);

package A0B.MPUXXXX.DMP612 is

   pragma Preelaborate;

   DMP_Sample_Rate : constant := 200;

   type Accelerometer_Data_Mode is (None, Raw);

   type Gyroscope_Data_Mode is (None, Raw, Calibrated);

   type Quaternion_Mode is (None, Quaternion_3, Quaternion_6);

   type Interrupt_Mode is (Gesture, Continuous);

   procedure Initialize;
   --  Copy unmodified firmware into the temporary buffer.

   --  procedure Upload_Firmware
   --    (Self    : in out Abstract_MPU_Sensor'Class;
   --     Success : in out Boolean);

   procedure Set_FIFO_Rate (FIFO_Rate : FIFO_Rate_Type);
   --  Must be called to enable FIFO.

   procedure Set_Interrupt_Mode (Mode : Interrupt_Mode);
   --  Default: Continuous

   procedure Enable_Features
     (Self                  : in out Abstract_MPU_Sensor'Class;
      Accelerometer         : Accelerometer_Data_Mode;
      Gyroscope             : Gyroscope_Data_Mode;
      Quaternion            : Quaternion_Mode;
      Gyroscope_Calibration : Boolean;
      Tap                   : Boolean;
      Android_Orientation   : Boolean);

   --  procedure Unpack_FIFO_Packet (Self : in out Abstract_MPU_Sensor'Class);

end A0B.MPUXXXX.DMP612;
