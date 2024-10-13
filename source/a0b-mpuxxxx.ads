--
--  Copyright (C) 2019-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  Common code for MPU-6000/MPU-6050/MPU-6500/MPU-9150/MPU-9250 family: the
--  Motion Processing Units
--
--  Note: This driver uses INT signal to initiate read of the sensor's data.
--  It is recommended to configure external interrupt to detect rising edge,
--  otherwise sensor doesn't acknowledge read operation of INT_STATUS register
--  from time to time.

pragma Restrictions (No_Elaboration_Code);

private with Interfaces;
private with System;

with A0B.Callbacks;
with A0B.EXTI;
with A0B.I2C.Device_Drivers_8;
private with A0B.Time;
private with A0B.Timer;
private with A0B.Types;

package A0B.MPUXXXX is

   pragma Preelaborate;

   type Gravitational_Acceleration is
     delta 1.0 / (2 ** 14) range -16.0 .. 16.0
       with Size => 32;

   type Angular_Velosity is
     delta 1.0 / (2 ** 15 * 1000 / 250) range -2_000.0 .. 2_000.0
       with Size => 32;

   type Temperature is delta 0.001 range -40.0 .. 85.0;

   type Abstract_MPU_Sensor
     (Controller : not null access A0B.I2C.I2C_Bus_Master'Class;
      Address    : A0B.I2C.Device_Address;
      --  Default device address is 16#68#. Sensor can be configured to
      --  use 16#69#.
      INT_Pin    : not null access A0B.EXTI.External_Interrupt_Line'Class)
      --  Pin must be configured to generate interrupt on falling edge.
     is abstract new A0B.I2C.Device_Drivers_8.I2C_Device_Driver with private;

   type Accelerometer_Range_Type is
     (FSR_2G,
      FSR_4G,
      FSR_8G,
      FSR_16G,
      Disabled);

   type Gyroscope_Range_Type is
     (FSR_250DPS,
      FSR_500DPS,
      FSR_1000DPS,
      FSR_2000DPS,
      Disabled);

   type Sample_Rate_Type is range 4 .. 1_000;

   type FIFO_Rate_Type is range 4 .. 200;

   procedure Configure
     (Self                : in out Abstract_MPU_Sensor'Class;
      Accelerometer_Range : Accelerometer_Range_Type;
      Gyroscope_Range     : Gyroscope_Range_Type;
      Temperature         : Boolean;
      Filter              : Boolean;
      Sample_Rate         : Sample_Rate_Type;
      Finished            : A0B.Callbacks.Callback;
      Success             : in out Boolean);
   --  Configure sensor in raw data mode, with low pass filter.
   --
   --  Rate of the Digital Low Pass Filter is selected automatically depending
   --  on given sample rate. Filter rate might be 188, 98, 42, 20, 10, or 5 Hz
   --  and at least two times less than sample rate.

   procedure Configure
     (Self      : in out Abstract_MPU_Sensor'Class;
      FIFO_Rate : FIFO_Rate_Type;
      Finished  : A0B.Callbacks.Callback;
      Success   : in out Boolean);
   --  Configure sensor in DMP mode.

   procedure Set_Data_Ready_Callback
     (Self     : in out Abstract_MPU_Sensor'Class;
      Callback : A0B.Callbacks.Callback);
   --  Sets callback to be called on receive of the new set of data.

   procedure Enable
     (Self     : in out Abstract_MPU_Sensor'Class;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean);
   --  Enables data load from the sensor.

   procedure Disable
     (Self     : in out Abstract_MPU_Sensor'Class;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean);
   --  Enables data load from the sensor.

private

   use A0B.I2C.Device_Drivers_8;

   package Registers is

      --  A_OFFS_USR (MPU6050: 6-11/06-0B),
      --  MPU6500 use different layout of registers with gaps (119-126/77-7E)

      type MPU6050_A_OFFS_USR_Register is record
         X_OFFS_USR : A0B.Types.Integer_16;
         Y_OFFS_USR : A0B.Types.Integer_16;
         Z_OFFS_USR : A0B.Types.Integer_16;
      end record
        with Pack,
             Object_Size          => 48,
             Bit_Order            => System.High_Order_First,
             Scalar_Storage_Order => System.High_Order_First;

      --  G_OFFS_USR (19-24/13-18)

      type G_OFFS_USR_Register is record
         X_OFFS_USR : A0B.Types.Integer_16;
         Y_OFFS_USR : A0B.Types.Integer_16;
         Z_OFFS_USR : A0B.Types.Integer_16;
      end record
        with Pack,
             Object_Size          => 48,
             Bit_Order            => System.High_Order_First,
             Scalar_Storage_Order => System.High_Order_First;

      --  SMPLRT_DIV (25/19)

      type SMPLRT_DIV_Register is record
         SMPLRT_DIV : Interfaces.Unsigned_8;
      end record
        with Object_Size => 8;

      --  CONFIG (26/1A)

      type DLPF_CFG_Type is mod 2 ** 3
        with Size => 3;
      --  Value 7 used by MPU6500/MPU9215 only

      type EXT_SYNC_SET_Type is
        (Disabled,
         TEMP_OUT_L,
         GYRO_XOUT_L,
         GYRO_YOUT_L,
         GYRO_ZOUT_L,
         ACCEL_XOUT_L,
         ACCEL_YOUT_L,
         ACCEL_ZOUT_L)
        with Size => 3;

      type CONFIG_Register is record
         DLPF_CFG          : DLPF_CFG_Type     := 0;
         EXT_SYNC_SET      : EXT_SYNC_SET_Type := Disabled;
         MPU6500_FIFO_MODE : Boolean           := False;
         Reserved_7        : Boolean           := False;
      end record
        with Pack, Object_Size => 8;

      --  GYRO_CONFIG (27/1B)

      type MPU6500_FCHOICE_B_Type is mod 2 ** 2
        with Size => 2;

      type GYRO_FS_SEL_Type is
        (G_250,
         G_500,
         G_1000,
         G_2000)
        with Size => 2;

      type GYRO_CONFIG_Register is record
         MPU6500_FCHOICE_B : MPU6500_FCHOICE_B_Type := 0;
         Reserved_2        : Boolean                := False;
         GYRO_FS_SEL       : GYRO_FS_SEL_Type       := G_250;
         ZG_ST             : Boolean                := False;
         YG_ST             : Boolean                := False;
         XG_ST             : Boolean                := False;
      end record
        with Pack, Object_Size => 8;

      --  ACCEL_CONFIG (28/1C)

      type ACCEL_FS_SEL_Type is
        (A_2,
         A_4,
         A_8,
         A_16)
        with Size => 2;

      type ACCEL_CONFIG_Register is record
         Reserved_0   : Boolean           := False;
         Reserved_1   : Boolean           := False;
         Reserved_2   : Boolean           := False;
         ACCEL_FS_SEL : ACCEL_FS_SEL_Type := A_2;
         ZA_ST        : Boolean           := False;
         YA_ST        : Boolean           := False;
         XA_ST        : Boolean           := False;
      end record
        with Pack, Object_Size => 8;

      --  MPU6500 ACCEL_CONFIG_2 (29/1D)

      type MPU6500_A_DLPF_CFG_Type is mod 2 ** 3
        with Size => 3;

      type MPU6500_ACCEL_CONFIG_2_Register is record
         A_DLPF_CFG     : MPU6500_A_DLPF_CFG_Type := 0;
         ACCEL_CHOICE_B : Boolean                 := False;
         Reserved_4     : Boolean                 := False;
         Reserved_5     : Boolean                 := False;
         FIFO_SIZE_1024 : Boolean                 := False;
         --  FIFO_SIZE_1024 is not documented.
         Reserved_7     : Boolean                 := False;
      end record
        with Pack, Object_Size => 8;

      --  FIFO_EN (35/23)

      type FIFO_EN_Register is record
         SLV0_FIFO_EN  : Boolean := False;
         SLV1_FIFO_EN  : Boolean := False;
         SLV2_FIFO_EN  : Boolean := False;
         ACCEL_FIFO_EN : Boolean := False;
         ZG_FIFO_EN    : Boolean := False;
         YG_FIFO_EN    : Boolean := False;
         XG_FIFO_EN    : Boolean := False;
         TEMP_FIFO_EN  : Boolean := False;
      end record
        with Pack, Object_Size => 8;

      --  INT_PIN_CFG (55/37)

      type INT_PIN_CFG_Register is record
         Reserved_0        : Boolean := False;
         I2C_BYPASS_EN     : Boolean := False;
         FSYNC_INT_MODE_EN : Boolean := False;
         ACTL_FSYNC        : Boolean := False;
         INT_ANYRD_2CLEAR  : Boolean := False;
         LATCH_INT_EN      : Boolean := False;
         OPEN              : Boolean := False;
         ACTL              : Boolean := False;
      end record
        with Pack, Object_Size => 8;

      --  INT_ENABLE (56/38)

      type INT_ENABLE_Register is record
         RAW_RDY_EN                  : Boolean := False;
         DMP_INT_EN                  : Boolean := False;
         Reserved_2                  : Boolean := False;
         I2C_MST_INT_EN_FSYNC_INT_EN : Boolean := False;
         --  MPU6050: I2C_MST_INT
         --  MPU6500: FSYNC_INT_EN
         FIFO_OFLOW_EN               : Boolean := False;
         DMP_ZMOT_EN                 : Boolean := False;
         MPU6500_WOM_EN_DMP_MOT_EN   : Boolean := False;
         --  MPU6500: Wake On Motion
         --  DMP AN: Motion Detection
         DMP_FF_EN                   : Boolean := False;
      end record
        with Pack, Object_Size => 8;

      --  INT_STATUS (58/3A)

      type INT_STATUS_Register is record
         DATA_RDY_INT               : Boolean := False;
         DMP_INT                    : Boolean := False;
         Reserved_2                 : Boolean := False;
         I2C_MST_INT_FSYNC_INT      : Boolean := False;
         --  MPU6050: I2C_MST_INT
         --  MPU6500: FSYNC_INT_EN
         FIFO_OFLOW_INT             : Boolean := False;
         DMP_ZMOT_INT               : Boolean := False;
         MPU6500_WOM_EN_DPM_MOT_INT : Boolean := False;
         --  MPU6500: Wake On Motion
         --  DMP AN: Motion Detection
         DMP_FF_INT                 : Boolean := False;
      end record
        with Pack, Object_Size => 8;

      --  ACCEL_OUT (59..64/3B..40)

      type ACCEL_OUT_Register is record
         XOUT : Interfaces.Integer_16;
         YOUT : Interfaces.Integer_16;
         ZOUT : Interfaces.Integer_16;
      end record
        with Pack,
             Object_Size          => 48,
             Bit_Order            => System.High_Order_First,
             Scalar_Storage_Order => System.High_Order_First;

      --  TEMP_OUT (65..66/41..42)

      type TEMP_OUT_Register is record
         TEMP_OUT : Interfaces.Integer_16;
      end record
        with Pack,
             Object_Size          => 16,
             Bit_Order            => System.High_Order_First,
             Scalar_Storage_Order => System.High_Order_First;

      --  GYRO_OUT (67..72/43..48)

      type GYRO_OUT_Register is record
         XOUT : Interfaces.Integer_16;
         YOUT : Interfaces.Integer_16;
         ZOUT : Interfaces.Integer_16;
      end record
        with Pack,
             Object_Size          => 48,
             Bit_Order            => System.High_Order_First,
             Scalar_Storage_Order => System.High_Order_First;

      --  SIGNAL_PATH_RESET (104/68)

      type SIGNAL_PATH_RESET_Register is record
         TEMP_Reset  : Boolean := False;
         ACCEL_Reset : Boolean := False;
         GYRO_Reset  : Boolean := False;
         Reserved_3  : Boolean := False;
         Reserved_4  : Boolean := False;
         Reserved_5  : Boolean := False;
         Reserved_6  : Boolean := False;
         Reserved_7  : Boolean := False;
      end record
        with Pack, Object_Size => 8;

      --  USER_CTRL (106/6A)

      type USER_CTRL_Register is record
         SIG_COND_RESET : Boolean := False;
         I2C_MST_RESET  : Boolean := False;
         FIFO_RESET     : Boolean := False;
         DMP_RESET      : Boolean := False;
         I2C_IF_DIS     : Boolean := False;
         I2C_MST_EN     : Boolean := False;
         FIFO_EN        : Boolean := False;
         DMP_EN         : Boolean := False;
      end record
        with Pack, Object_Size => 8;

      --  PWR_MGM_1 (107/6B)

      type CLKSEL_Type is    --     MPU6050              MPU6500
        (Internal,           --  Internal 8MHz        Internal 20MHz
         PLL_X,              --  PLL X gyro           Internal/PLL auto
         PLL_Y,              --  PLL Y guro           Internal/PLL auto
         PLL_Z,              --  PLL Z gyro           Internal/PLL auto
         PLL_32_768_K,       --  External 32.768kHz   Internal/PLL auto
         PLL_19_2_M,         --  External 19.2MHz     Internal/PLL auto
         MPU6500_Internal,   --                       Internal 20MHz
         Stop)
        with Size => 3;

      type PWR_MGMT_1_Register is record
         CLKSEL           : CLKSEL_Type := Internal;
         TEMP_DIS         : Boolean     := False;
         --  MPU9250 name it as PD_PTAT: Power Disable Proportional To Absolute
         --  Temperature sensor
         GYRO_STANDBY     : Boolean     := False;
         CYCLE            : Boolean     := False;
         SLEEP            : Boolean     := False;
         DEVICE_RESET     : Boolean     := False;
      end record
        with Pack, Object_Size => 8;

      --  PWR_MGMT_2 (108/6C)

      type LP_WAKE_CTRL_Type is mod 2 ** 2
        with Size => 2;

      type PWR_MGMT_2_Register is record
         STBY_ZG      : Boolean := False;
         STBY_YG      : Boolean := False;
         STBY_XG      : Boolean := False;
         STBY_ZA      : Boolean := False;
         STBY_YA      : Boolean := False;
         STBY_XA      : Boolean := False;
         LP_WAKE_CTRL : LP_WAKE_CTRL_Type := 0;
         --  Not supported by MPU9250
      end record
        with Pack, Object_Size => 8;

   --     --  DMP: BANK_SEL (109..110/6D..6E)
   --
   --     type BANK_SEL_Register is record
   --        Address : Interfaces.Unsigned_16;
   --     end record
   --       with Pack,
   --            Object_Size          => 16,
   --            Bit_Order            => System.High_Order_First,
   --            Scalar_Storage_Order => System.High_Order_First;
   --
   --     --  DMP: PRGM_START (112..113/70..71)
   --
   --     type PRGM_START_Register is record
   --        Address : Interfaces.Unsigned_16;
   --     end record
   --       with Pack,
   --            Object_Size          => 16,
   --            Bit_Order            => System.High_Order_First,
   --            Scalar_Storage_Order => System.High_Order_First;

      --  FIFO_COUNT (114-115/72-73)

      type FIFO_COUNT_Register is record
         Value : A0B.Types.Unsigned_16;
      end record
        with Pack,
             Object_Size          => 16,
             Bit_Order            => System.High_Order_First,
             Scalar_Storage_Order => System.High_Order_First;

      --  DMP_QUATERNION

      type DMP_QUAT_OUT_Register is record
         Q0 : Interfaces.Integer_32;
         Q1 : Interfaces.Integer_32;
         Q2 : Interfaces.Integer_32;
         Q3 : Interfaces.Integer_32;
      end record
        with Pack,
             Object_Size          => 128,
             Bit_Order            => System.High_Order_First,
             Scalar_Storage_Order => System.High_Order_First;

   end Registers;

   --  AK8975_Address : constant BBF.I2C.Device_Address := 16#0C#;

   MPU6050_WHOAMI : constant := 16#68#;
   MPU6500_WHOAMI : constant := 16#70#;
   MPU9250_WHOAMI : constant := 16#71#;

   MPU6050_XA_OFFS_USRH           : constant Register_Address := 16#06#;
   --  MPU6050_XA_OFFS_USRL           : constant Register_Address := 16#07#;
   MPU6050_YA_OFFS_USRH           : constant Register_Address := 16#08#;
   --  MPU6050_YA_OFFS_USRL           : constant Register_Address := 16#09#;
   MPU6050_ZA_OFFS_USRH           : constant Register_Address := 16#0A#;
   --  MPU6050_ZA_OFFS_USRL           : constant Register_Address := 16#0B#;
   XA_OFFS_USR_Length             : constant                  := 2;
   YA_OFFS_USR_Length             : constant                  := 2;
   ZA_OFFS_USR_Length             : constant                  := 2;

   XG_OFFS_USRH_Address           : constant Register_Address := 16#13#;
   --  XG_OFFS_USRL_Address           : constant Register_Address := 16#14#;
   --  YG_OFFS_USRH_Address           : constant Register_Address := 16#15#;
   --  YG_OFFS_USRL_Address           : constant Register_Address := 16#16#;
   --  ZG_OFFS_USRH_Address           : constant Register_Address := 16#17#;
   --  ZG_OFFS_USRL_Address           : constant Register_Address := 16#18#;
   G_OFFS_USR_Length              : constant                  := 6;
   SMPLRT_DIV_Address             : constant Register_Address := 16#19#;
   --  CONFIG_Address                 : constant Register_Address := 16#1A#;
   --  GYRO_CONFIG_Address            : constant Register_Address := 16#1B#;
   --  ACCEL_CONFIG_Address           : constant Register_Address := 16#1C#;
   --  MPU6500_ACCEL_CONFIG_2_Address : constant Register_Address := 16#1D#;

   FIFO_EN_Address                : constant Register_Address := 16#23#;

   INT_PIN_CFG_Address            : constant Register_Address := 16#37#;
   INT_ENABLE_Address             : constant Register_Address := 16#38#;

   INT_STATUS_Address             : constant Register_Address := 16#3A#;

   --  ACCEL_OUT_Address              : constant Register_Address := 16#3B#;
   ACCEL_OUT_Length               : constant                  := 6;
   --  TEMP_OUT_Address               : constant Register_Address := 16#41#;
   TEMP_OUT_Length                : constant                  := 2;
   --  GYRO_OUT_Address               : constant Register_Address := 16#43#;
   GYRO_OUT_Length                : constant                  := 6;

   SIGNAL_PATH_RESET_Address      : constant Register_Address := 16#68#;

   USER_CTRL_Address              : constant Register_Address := 16#6A#;
   PWR_MGMT_1_Address             : constant Register_Address := 16#6B#;
   --  PWR_MGMT_2_Address             : constant Register_Address := 16#6C#;
   --  DMP_BANK_SEL_Address           : constant Register_Address := 16#6D#;
   --  DMP_BANK_SEL_Length            : constant                  := 2;
   --  DMP_MEM_R_W_Address            : constant Register_Address := 16#6F#;
   --  DMP_PRGM_START_Address         : constant Register_Address := 16#70#;
   --  DMP_PRGM_START_Length          : constant                  := 2;
   FIFO_COUNT_Address             : constant Register_Address := 16#72#;
   FIFO_COUNT_Length              : constant                  := 2;
   FIFO_R_W_Address               : constant Register_Address := 16#74#;
   WHO_AM_I_Address               : constant Register_Address := 16#75#;

   MPU6500_XA_OFFS_USRH           : constant Register_Address := 16#77#;
   MPU6500_XA_OFFS_USRL           : constant Register_Address := 16#78#;

   MPU6500_YA_OFFS_USRH           : constant Register_Address := 16#7A#;
   MPU6500_YA_OFFS_USRL           : constant Register_Address := 16#7B#;

   MPU6500_ZA_OFFS_USRH           : constant Register_Address := 16#7D#;
   MPU6500_ZA_OFFS_USRL           : constant Register_Address := 16#7E#;

   --  DMP_QUAT_OUT_Length            : constant                  := 16;

   type Raw_Data is record
      ACCEL     : Registers.ACCEL_OUT_Register;
      TEMP      : Registers.TEMP_OUT_Register;
      GYRO      : Registers.GYRO_OUT_Register;
      QUAT      : Registers.DMP_QUAT_OUT_Register;
      Timestamp : A0B.Time.Monotonic_Time;
   end record
     with Object_Size => 304;

   type Raw_Data_Array is array (Boolean) of Raw_Data;

   type Calibration_Data is record
      Accelerometer : Registers.MPU6050_A_OFFS_USR_Register;
      Gyroscope     : Registers.G_OFFS_USR_Register;
      --  Magnitomter
   end record;

   type States is
     (Initial,
      Initialization_WHOAMI_Check,
      Initialization_Device_Reset,
      Initialization_Device_Reset_Delay,
      Initialization_Signal_Path_Reset,
      Initialization_Signal_Path_Reset_Delay,
      Initialization_Wakeup,
      Configuration_CONFIG,
      Configuration_PWR_MGMT,
      Configuration_Delay,
      Enable_INT_ENABLE_Disable,
      Enable_FIFO_EN_Disable,
      Enable_USER_CTRL_Disable,
      Enable_Reset,
      Enable_Reset_Delay,
      Enable_USER_CTRL_Enable,
      Enable_INT_Enable,
      Enable_FIFO_EN_Enable,
      Disable_INT_ENABLE_Disable,
      Disable_FIFO_EN_Disable,
      Disable_USER_CTRL_Disable,
      Interrupt_INT_STATUS,
      Interrupt_FIFO_COUNT,
      Interrupt_FIFO_R_W,
      Get_XA_OFFS_USR,
      Get_YA_OFFS_USR,
      Get_ZA_OFFS_USR,
      Get_G_OFFS_USR,
      Set_XA_OFFS_USR,
      Set_YA_OFFS_USR,
      Set_ZA_OFFS_USR,
      Set_G_OFFS_USR,
      Ready);

   type Abstract_MPU_Sensor
     (Controller : not null access A0B.I2C.I2C_Bus_Master'Class;
      Address    : A0B.I2C.Device_Address;
      INT_Pin    : not null access A0B.EXTI.External_Interrupt_Line'Class) is
     abstract new A0B.I2C.Device_Drivers_8.I2C_Device_Driver
                       (Controller => Controller,
                        Address    => Address)
     with record
      State                     : States  := Initial;
      Initialized               : Boolean := False;
      Data_Ready                : A0B.Callbacks.Callback;
      --  Callback to be called on receive of the new data package.

      WHOAMI                    : A0B.Types.Unsigned_8;

      Accelerometer_Enabled     : Boolean := False;
      Accelerometer_Scale       : Natural range 0 .. 3 := 0;
      Gyroscope_Enabled         : Boolean := False;
      Gyroscope_Scale           : Natural range 0 .. 3 := 0;
      Temperature_Enabled       : Boolean := False;

      DMP_Enabled               : Boolean := False;
      DMP_Accelerometer_Enabled : Boolean := False;
      DMP_Gyroscope_Enabled     : Boolean := False;
      DMP_Quaternion_Enabled    : Boolean := False;
      DMP_Gesture_Enabled       : Boolean := False;

      FIFO_Packet_Size          : A0B.Types.Unsigned_16 := 0;
      --  Size of the FIFO packet to download and decode. It depends of
      --  configuration of the sensor.

      FIFO_Remaining_Size       : A0B.Types.Unsigned_16 := 0;
      --  Number of bytes remaining in the FIFO.

      User_Bank                 : Boolean := False with Volatile;
      Raw_Data                  : Raw_Data_Array;
      --  Two banks of collected information: one is used by the user, and
      --  another one asynchronous read handler. Banks are switched by the
      --  handler after successful load of new packet of data.

      Timeout                   : aliased A0B.Timer.Timeout_Control_Block;

      Finished                  : A0B.Callbacks.Callback;
      --  Callback to execute then operation finished.

      Calibration_Buffer        : access Calibration_Data;
      --  Application defined location of the calibration data.

      Transfer_Buffer           : A0B.I2C.Unsigned_8_Array (0 .. 31);
      --  Storage for IO operations:
      --   - firmware upload buffer
      --     - size should be power of two to avoid cross of bank boundary
      --   - FIFO packet buffer, size should be is enough to store
      --     - accelerometer data (6 bytes)
      --     - temperature data (2 bytes)
      --     - gyroscope data (6 bytes)
      --   - DMP FIFO packet buffer, size should be is enough to store
      --     - quaternion data (16 bytes)
      --     - accelerometer data (6 bytes)
      --     - gyroscope data (6 bytes)
      --     - questure data (4 bytes)
      Transfer_Status           : aliased
        A0B.I2C.Device_Drivers_8.Transaction_Status;
   end record;

   procedure Internal_Initialize
     (Self     : in out Abstract_MPU_Sensor'Class;
      WHOAMI   : A0B.Types.Unsigned_8;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean);
   --  First step of the initialization procedure. Probe controller and check
   --  chip identifier.

   not overriding function Is_6500_9250
     (Self : Abstract_MPU_Sensor) return Boolean is (raise Program_Error);

   not overriding function To_Temperature
     (Self : Abstract_MPU_Sensor;
      Raw  : Interfaces.Integer_16) return Temperature is
        (raise Program_Error);

   function To_Gravitational_Acceleration
     (Self : Abstract_MPU_Sensor'Class;
      Raw  : Interfaces.Integer_16) return Gravitational_Acceleration;

   function To_Angular_Velosity
     (Self : Abstract_MPU_Sensor'Class;
      Raw  : Interfaces.Integer_16) return Angular_Velosity;

   procedure Get_XA_OFFS_USR_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure Set_XA_OFFS_USR_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   --  -------------------
   --  --  API for DMP  --
   --  -------------------
   --
   --  procedure Upload_Firmware
   --    (Self     : in out Abstract_MPU_Sensor'Class;
   --     Firmware : A0B.I2C.Unsigned_8_Array;
   --     Address  : Interfaces.Unsigned_16;
   --     Success  : in out Boolean);
   --  --  Upload firmware to sensor. It is synchronous operation.
   --
   --  procedure Write_DMP_Memory
   --    (Self    : in out Abstract_MPU_Sensor'Class;
   --     Address : Interfaces.Unsigned_16;
   --     Data    : A0B.I2C.Unsigned_8_Array;
   --     Success : in out Boolean);
   --
   --  procedure Read_DMP_Memory
   --    (Self    : in out Abstract_MPU_Sensor'Class;
   --     Address : Interfaces.Unsigned_16;
   --     Data    : out A0B.I2C.Unsigned_8_Array;
   --     Success : in out Boolean);

end A0B.MPUXXXX;
