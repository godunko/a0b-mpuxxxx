--
--  Copyright (C) 2019-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

pragma Restrictions (No_Elaboration_Code);
pragma Ada_2022;

with Ada.Unchecked_Conversion;
--  with System.Address_To_Access_Conversions;
with System.Storage_Elements;

with A0B.Callbacks.Generic_Non_Dispatching;
with A0B.MPUXXXX.DMP612;
with A0B.Time.Clock;

package body A0B.MPUXXXX is

   type CONFIG_Resgisters is record
      SMPLRT_DIV     : Registers.SMPLRT_DIV_Register;
      CONFIG         : Registers.CONFIG_Register;
      GYRO_CONFIG    : Registers.GYRO_CONFIG_Register;
      ACCEL_CONFIG   : Registers.ACCEL_CONFIG_Register;
      ACCEL_CONFIG_2 : Registers.MPU6500_ACCEL_CONFIG_2_Register;
   end record
     with Pack, Object_Size => 40;

   type PWR_MGMT_Registers is record
      PWR_MGMT_1 : Registers.PWR_MGMT_1_Register;
      PWR_MGMT_2 : Registers.PWR_MGMT_2_Register;
   end record
     with Pack, Object_Size => 16;

   type INT_Registers is record
      INT_PIN_CFG : Registers.INT_PIN_CFG_Register;
      INT_ENABLE  : Registers.INT_ENABLE_Register;
   end record
     with Object_Size => 16;

   procedure Compute_CONFIG
     (Self                : in out Abstract_MPU_Sensor'Class;
      CONFIG              : out CONFIG_Resgisters;
      Accelerometer_Range : Accelerometer_Range_Type;
      Gyroscope_Range     : Gyroscope_Range_Type;
      Filter              : Boolean;
      Sample_Rate         : Sample_Rate_Type);
   --  Compute value of CONFIG registers on given parameters.

   procedure On_Interrupt (Self : in out Abstract_MPU_Sensor'Class);

   package On_Interrupt_Callbacks is
     new A0B.Callbacks.Generic_Non_Dispatching
           (Abstract_MPU_Sensor, On_Interrupt);

   --  procedure On_FIFO_Count_Read (Closure : System.Address);
   --
   --  procedure On_FIFO_Data_Read (Closure : System.Address);
   --
   --  package Conversions is
   --    new System.Address_To_Access_Conversions
   --          (Object => Abstract_MPU_Sensor'Class);

   procedure On_Operation_Finished (Self : in out Abstract_MPU_Sensor'Class);

   package On_Operation_Finished_Callbacks is
     new A0B.Callbacks.Generic_Non_Dispatching
           (Abstract_MPU_Sensor, On_Operation_Finished);

   procedure WHOAMI_Check_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      WHOAMI  : A0B.Types.Unsigned_8;
      Success : in out Boolean);

   procedure WHOAMI_Check_Complete
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure Device_Reset_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure Device_Reset_Delay_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure Signal_Path_Reset_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure Signal_Path_Reset_Delay_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure Wakeup_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure CONFIG_Initiate
     (Self                : in out Abstract_MPU_Sensor'Class;
      Accelerometer_Range : Accelerometer_Range_Type;
      Gyroscope_Range     : Gyroscope_Range_Type;
      Filter              : Boolean;
      Sample_Rate         : Sample_Rate_Type;
      Success             : in out Boolean);

   procedure PWR_MGMT_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure Configuration_Delay_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure INT_ENABLE_Disable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure FIFO_EN_Disable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure USER_CTRL_Disable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure Reset_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure Reset_Delay_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure USER_CTRL_Enable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure INT_Enable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure FIFO_EN_Enable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure INT_STATUS_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure INT_STATUS_Complete_FIFO_COUNT_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure FIFO_COUNT_Complete_FIFO_R_W_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   procedure FIFO_R_W_Complete
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean);

   --  DMP_Bank_Size : constant := 256;

   --------------------
   -- Compute_CONFIG --
   --------------------

   procedure Compute_CONFIG
     (Self                : in out Abstract_MPU_Sensor'Class;
      CONFIG              : out CONFIG_Resgisters;
      Accelerometer_Range : Accelerometer_Range_Type;
      Gyroscope_Range     : Gyroscope_Range_Type;
      Filter              : Boolean;
      Sample_Rate         : Sample_Rate_Type)
   is
      pragma Unreferenced (Self);

      use type Interfaces.Unsigned_16;

      SMPLRT_DIV         : constant Interfaces.Unsigned_8 :=
        Interfaces.Unsigned_8
          ((if Filter then 1_000 else 8_000) / Sample_Rate - 1);
      --  MPU6500 has additional 8_000 and 32_000 modes, however, this value is
      --  used only when DLPF in 1 .. 6.
      Actual_Sample_Rate : constant Sample_Rate_Type :=
        Sample_Rate_Type
          ((if Filter then 1_000 else 8_000)
              / (1 + Interfaces.Unsigned_16 (SMPLRT_DIV)));

   begin
      CONFIG :=
        (SMPLRT_DIV     =>
           (SMPLRT_DIV => SMPLRT_DIV),
           --  MPU6050: Gyro rate is 8k when CONFIG:DLPF_CFG = 0, and
           --  1k overwise. MPU6500: Depends from CONFIG:DLPF_CFG and
           --  GYRO_CFG:FCHOICE_B, looks compatible with allowed MPU6050
           --  values.
         CONFIG         =>
           (DLPF_CFG          =>
                (if not Filter then 0
                 elsif Actual_Sample_Rate >= 188 * 2 then 1
                 elsif Actual_Sample_Rate >=  98 * 2 then 2
                 elsif Actual_Sample_Rate >=  42 * 2 then 3
                 elsif Actual_Sample_Rate >=  20 * 2 then 4
                 elsif Actual_Sample_Rate >=  10 * 2 then 5
                                                     else 6),
            --  MPU6500 support value 7 to bypass filter, not implemented.
            EXT_SYNC_SET      => Registers.Disabled,
            MPU6500_FIFO_MODE => False,
            others            => False),
         --  Enable DLPF_CFG, rate will be about 180 Hz.
         GYRO_CONFIG    =>
           (MPU6500_FCHOICE_B => 0,
            GYRO_FS_SEL       =>
              (case Gyroscope_Range is
                  when FSR_250DPS  => Registers.G_250,
                  when FSR_500DPS  => Registers.G_500,
                  when FSR_1000DPS => Registers.G_1000,
                  when FSR_2000DPS => Registers.G_2000,
                  when Disabled    => Registers.GYRO_FS_SEL_Type'First),
            others            => False),
         ACCEL_CONFIG   =>
           (ACCEL_FS_SEL =>
                (case Accelerometer_Range is
                    when FSR_2G   => Registers.A_2,
                    when FSR_4G   => Registers.A_4,
                    when FSR_8G   => Registers.A_8,
                    when FSR_16G  => Registers.A_16,
                    when Disabled => Registers.ACCEL_FS_SEL_Type'First),
            others       => False),
         ACCEL_CONFIG_2 =>
           (A_DLPF_CFG     =>
                (if not Filter then 0
                 elsif Actual_Sample_Rate >= 188 * 2 then 1
                 elsif Actual_Sample_Rate >=  98 * 2 then 2
                 elsif Actual_Sample_Rate >=  42 * 2 then 3
                 elsif Actual_Sample_Rate >=  20 * 2 then 4
                 elsif Actual_Sample_Rate >=  10 * 2 then 5
                                                     else 6),
            ACCEL_CHOICE_B => False,
            FIFO_SIZE_1024 => True,
            --  MPU6500 shares 4kB of memory between the DMP and the FIFO.
            --  Since the first 3kB are needed by the DMP, we'll use the
            --  last 1kB for the FIFO.
            others         => False));
         --  This register is available on MPU6500/9250 only. Selected values
         --  run accelerometer at about 180 Hz, like gyro.

      --  Configuration of SMPLRT_DIV is set to lower gyro rate on MPU6050 to
      --  rate of the accelerometer.
      --
      --  DLPF is enabled and DLPF_CFG/FCHOICE_B set to have gyro rate 184 Hz.
      --  It is possible to lower gyro rate and have it close to accelerometer
      --  rate.
      --
      --  On MPU6500/9250 accelerometer configured to 188 Hz rate. It is
      --  separate rergister on these sensors, on MPU6050/9150 when DLPF
      --  is configured it applies to both gyro and accelerometer.
      --
      --  MPU6500/9250 has more modes, but they are not compatible with
      --  MPU6050/9150 and not useful for my purposes.
   end Compute_CONFIG;

   ---------------------
   -- CONFIG_Initiate --
   ---------------------

   procedure CONFIG_Initiate
     (Self                : in out Abstract_MPU_Sensor'Class;
      Accelerometer_Range : Accelerometer_Range_Type;
      Gyroscope_Range     : Gyroscope_Range_Type;
      Filter              : Boolean;
      Sample_Rate         : Sample_Rate_Type;
      Success             : in out Boolean)
   is
      CONFIG   : CONFIG_Resgisters
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer   : A0B.I2C.Unsigned_8_Array
                   (0 .. (if Self.Is_6500_9250 then 4 else 3))
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Configuration_CONFIG;

      Self.Compute_CONFIG
        (CONFIG, Accelerometer_Range, Gyroscope_Range, Filter, Sample_Rate);

      if Self.DMP_Enabled then
         CONFIG.CONFIG.DLPF_CFG := 3;
         --  Application Notes for DMP recommends to use 42 Hz, while library
         --  configure it to 98 Hz. Use value from Application Notes.
      end if;

      Self.Write
        (Address      => SMPLRT_DIV_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end CONFIG_Initiate;

   ----------------------------------
   -- Configuration_Delay_Initiate --
   ----------------------------------

   procedure Configuration_Delay_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      pragma Warnings (Off, Success);

   begin
      if not Success then
         return;
      end if;

      Self.State := Configuration_Delay;

      A0B.Timer.Enqueue
        (Timeout  => Self.Timeout,
         Callback => On_Operation_Finished_Callbacks.Create_Callback (Self),
         T        => A0B.Time.Milliseconds (50));
   end Configuration_Delay_Initiate;

   ---------------
   -- Configure --
   ---------------

   procedure Configure
     (Self                : in out Abstract_MPU_Sensor'Class;
      Accelerometer_Range : Accelerometer_Range_Type;
      Gyroscope_Range     : Gyroscope_Range_Type;
      Temperature         : Boolean;
      Filter              : Boolean;
      Sample_Rate         : Sample_Rate_Type;
      Finished            : A0B.Callbacks.Callback;
      Success             : in out Boolean)
   is
      use type A0B.Types.Unsigned_16;

   begin
      if not Success or Self.State /= Ready then
         Success := False;

         return;
      end if;

      Self.Finished              := Finished;
      Self.Accelerometer_Enabled := Accelerometer_Range /= Disabled;
      Self.Gyroscope_Enabled     := Gyroscope_Range /= Disabled;
      Self.Temperature_Enabled   := Temperature;
      Self.DMP_Enabled           := False;
      Self.FIFO_Packet_Size :=
        (if Self.Accelerometer_Enabled then 6 else 0)
          + (if Self.Gyroscope_Enabled then 6 else 0)
          + (if Self.Temperature_Enabled then 2 else 0);

      Self.CONFIG_Initiate
        (Accelerometer_Range => Accelerometer_Range,
         Gyroscope_Range     => Gyroscope_Range,
         Filter              => Filter,
         Sample_Rate         => Sample_Rate,
         Success             => Success);
   end Configure;

   ---------------
   -- Configure --
   ---------------

   procedure Configure
     (Self      : in out Abstract_MPU_Sensor'Class;
      FIFO_Rate : FIFO_Rate_Type;
      Finished  : A0B.Callbacks.Callback;
      Success   : in out Boolean) is
   begin
      if not Success or Self.State /= Ready then
         Success := False;

         return;
      end if;

      Self.Finished              := Finished;
      Self.Accelerometer_Enabled := True;
      Self.Gyroscope_Enabled     := True;
      Self.Temperature_Enabled   := True;
      Self.DMP_Enabled           := True;

      DMP612.Initialize;
      DMP612.Set_FIFO_Rate (FIFO_Rate);
      DMP612.Set_Interrupt_Mode (DMP612.Continuous);
      DMP612.Enable_Features
        (Self                  => Self,
         Accelerometer         => DMP612.Raw,
         Gyroscope             => DMP612.Calibrated,
         Quaternion            => DMP612.Quaternion_6,
         Gyroscope_Calibration => True,
         Tap                   => False,
         Android_Orientation   => False);

      Self.CONFIG_Initiate
        (Accelerometer_Range => FSR_2G,
         Gyroscope_Range     => FSR_2000DPS,
         Filter              => True,
         Sample_Rate         => DMP612.DMP_Sample_Rate,
         Success             => Success);

   --     --  BLACK MAGIC!
   --
   --     DMP612.Upload_Firmware (Self, Success);
   --     DMP612.Set_FIFO_Rate (Self, FIFO_Rate);
   --     DMP612.Set_Interrupt_Mode (Self, DMP612.Continuous);
   --     DMP612.Enable_Features
   --       (Self                  => Self,
   --        Accelerometer         => DMP612.Raw,
   --        Gyroscope             => DMP612.Calibrated,
   --        Quaternion            => DMP612.Quaternion_6,
   --        Gyroscope_Calibration => True,
   --        Tap                   => False,
   --        Android_Orientation   => False);
   end Configure;

   ---------------------------------
   -- Device_Reset_Delay_Initiate --
   ---------------------------------

   procedure Device_Reset_Delay_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      pragma Warnings (Off, Success);

   begin
      if not Success then
         return;
      end if;

      Self.State := Initialization_Device_Reset_Delay;

      A0B.Timer.Enqueue
        (Timeout  => Self.Timeout,
         Callback => On_Operation_Finished_Callbacks.Create_Callback (Self),
         T        => A0B.Time.Milliseconds (100));
   end Device_Reset_Delay_Initiate;

   ---------------------------
   -- Device_Reset_Initiate --
   ---------------------------

   procedure Device_Reset_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      PWR_MGMT_1 : Registers.PWR_MGMT_1_Register
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer     : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Initialization_Device_Reset;

      PWR_MGMT_1 :=
        (DEVICE_RESET => True,
         CLKSEL       => Registers.Internal,
         others       => <>);

      Self.Write
        (Address      => PWR_MGMT_1_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end Device_Reset_Initiate;

   ------------
   -- Enable --
   ------------

   procedure Enable
     (Self     : in out Abstract_MPU_Sensor'Class;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean) is
   begin
      if not Success or Self.State /= Ready then
         Success := False;
      end if;

      Self.Finished := Finished;

      Self.INT_ENABLE_Disable_Initiate (Success);
   end Enable;

   -------------------------------------------
   -- FIFO_COUNT_Complete_FIFO_R_W_Initiate --
   -------------------------------------------

   procedure FIFO_COUNT_Complete_FIFO_R_W_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      use type A0B.Types.Unsigned_16;

      Amount : constant Registers.FIFO_COUNT_Register
        with Import, Address => Self.Transfer_Buffer (0)'Address;
      Buffer : A0B.I2C.Unsigned_8_Array
                 (0 .. A0B.Types.Unsigned_32 (Self.FIFO_Packet_Size - 1))
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      if Amount.Value < Self.FIFO_Packet_Size then
         --  Not enough data available.

         Self.State := Ready;

         return;
      end if;

      Self.State := Interrupt_FIFO_R_W;

      Self.Read
        (Address      => FIFO_R_W_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end FIFO_COUNT_Complete_FIFO_R_W_Initiate;

   ------------------------------
   -- FIFO_EN_Disable_Initiate --
   ------------------------------

   procedure FIFO_EN_Disable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      FIFO_EN : Registers.FIFO_EN_Register
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer  : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Enable_FIFO_EN_Disable;

      FIFO_EN := (others => False);

      Self.Write
        (Address      => FIFO_EN_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end FIFO_EN_Disable_Initiate;

   -----------------------------
   -- FIFO_EN_Enable_Initiate --
   -----------------------------

   procedure FIFO_EN_Enable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      FIFO_EN :  Registers.FIFO_EN_Register
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer    : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Enable_FIFO_EN_Enable;

      FIFO_EN  :=
        (ACCEL_FIFO_EN => Self.Accelerometer_Enabled and not Self.DMP_Enabled,
         XG_FIFO_EN    => Self.Gyroscope_Enabled and not Self.DMP_Enabled,
         YG_FIFO_EN    => Self.Gyroscope_Enabled and not Self.DMP_Enabled,
         ZG_FIFO_EN    => Self.Gyroscope_Enabled and not Self.DMP_Enabled,
         TEMP_FIFO_EN  => Self.Temperature_Enabled and not Self.DMP_Enabled,
         others        => False);

      Self.Write
        (Address      => FIFO_EN_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end FIFO_EN_Enable_Initiate;

   -----------------------
   -- FIFO_R_W_Complete --
   -----------------------

   procedure FIFO_R_W_Complete
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      pragma Warnings (Off, Success);

      use type System.Storage_Elements.Storage_Offset;

      Data   : Raw_Data renames Self.Raw_Data (not Self.User_Bank);
      Offset : System.Storage_Elements.Storage_Offset := 0;

   begin
      if Self.DMP_Enabled then
         raise Program_Error;
   --        DMP612.Unpack_FIFO_Packet (Self.all);
   --
   --        return;
      end if;

      Self.State := Ready;

      if Self.Accelerometer_Enabled then
         declare
            Aux : constant Registers.ACCEL_OUT_Register
              with Import, Address => Self.Transfer_Buffer'Address + Offset;

         begin
            Data.ACCEL := Aux;
            Offset     := Offset + ACCEL_OUT_Length;
         end;

      else
         Data.ACCEL := (others => 0);
      end if;

      if Self.Temperature_Enabled then
         declare
            Aux : constant Registers.TEMP_OUT_Register
              with Import, Address => Self.Transfer_Buffer'Address + Offset;

         begin
            Data.TEMP := Aux;
            Offset    := Offset + TEMP_OUT_Length;
         end;

      else
         Data.TEMP := (others => 0);
      end if;

      if Self.Gyroscope_Enabled then
         declare
            Aux : constant Registers.GYRO_OUT_Register
              with Import, Address => Self.Transfer_Buffer'Address + Offset;

         begin
            Data.GYRO := Aux;
            Offset    := Offset + GYRO_OUT_Length;
         end;

      else
         Data.GYRO := (others => 0);
      end if;

      Data.Timestamp := A0B.Time.Clock;
      Self.User_Bank := not @;
   end FIFO_R_W_Complete;

   ---------------------------------
   -- INT_ENABLE_Disable_Initiate --
   ---------------------------------

   procedure INT_ENABLE_Disable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      INT_ENABLE : Registers.INT_ENABLE_Register
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer     : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Enable_INT_ENABLE_Disable;

      INT_ENABLE := (others => False);

      Self.Write
        (Address      => INT_ENABLE_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end INT_ENABLE_Disable_Initiate;

   -------------------------
   -- INT_Enable_Initiate --
   -------------------------

   procedure INT_Enable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      INT    : INT_Registers
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer : A0B.I2C.Unsigned_8_Array (0 .. 1)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Enable_INT_Enable;

      INT :=
        (INT_PIN_CFG  =>
           (ACTL             => True,
            LATCH_INT_EN     => False,
            INT_ANYRD_2CLEAR => True,
            others           => <>),
         INT_ENABLE   =>
           (RAW_RDY_EN => not Self.DMP_Enabled,
            DMP_INT_EN => Self.DMP_Enabled,
            others     => False));

      Self.Write
        (Address      => INT_PIN_CFG_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end INT_Enable_Initiate;

   ---------------------------------------------
   -- INT_STATUS_Complete_FIFO_COUNT_Initiate --
   ---------------------------------------------

   procedure INT_STATUS_Complete_FIFO_COUNT_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      use type A0B.Types.Unsigned_32;

      INT_STATUS : constant Registers.INT_STATUS_Register
        with Import, Address => Self.Transfer_Buffer (0)'Address;
      Buffer     : A0B.I2C.Unsigned_8_Array (0 .. FIFO_COUNT_Length - 1)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      if INT_STATUS.FIFO_OFLOW_INT then
         --  FIFO overflow, operations should be shutdown and FIFO is
         --  restarted.
         --  XXX Not implemented.

         raise Program_Error with "MPU6xxx FIFO OVERFLOW";
      end if;

      if (INT_STATUS.DATA_RDY_INT and not Self.DMP_Enabled)
        or (INT_STATUS.DMP_INT and Self.DMP_Enabled)
      then
         --  Initiate load of amount of data available in FIFO.

         Self.State := Interrupt_FIFO_COUNT;

         Self.Read
           (Address      => FIFO_COUNT_Address,
            Buffer       => Buffer,
            Status       => Self.Transfer_Status,
            On_Completed =>
              On_Operation_Finished_Callbacks.Create_Callback (Self),
            Success      => Success);

      else
         raise Program_Error;
      end if;
   end INT_STATUS_Complete_FIFO_COUNT_Initiate;

   -------------------------
   -- INT_STATUS_Initiate --
   -------------------------

   procedure INT_STATUS_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      Buffer : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Interrupt_INT_STATUS;

      Self.Read
        (Address      => INT_STATUS_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end INT_STATUS_Initiate;

   -------------------------
   -- Internal_Initialize --
   -------------------------

   procedure Internal_Initialize
     (Self     : in out Abstract_MPU_Sensor'Class;
      WHOAMI   : A0B.Types.Unsigned_8;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean) is
   begin
      Self.Initialized := False;

      if not Success then
         return;
      end if;

      Self.Finished := Finished;
      Self.INT_Pin.Set_Callback (On_Interrupt_Callbacks.Create_Callback (Self));

   --     --  Do controller's probe.
   --
   --     Success := Self.Bus.Probe (Self.Device);
   --
   --     if not Success then
   --        return;
   --     end if;
   --

      Self.WHOAMI_Check_Initiate (WHOAMI, Success);
   end Internal_Initialize;

   ------------------
   -- On_Interrupt --
   ------------------

   procedure On_Interrupt (Self : in out Abstract_MPU_Sensor'Class) is
      Success : Boolean := True;

   begin
      --  Initiate read of INT_STATUS register.
      --
      --  Unfortunately, read of FIFO_COUNT is not enough: sometimes FIFO_COUNT
      --  has size of block, but when block is downloaded it contains all zero
      --  bytes. It is happend when INT_STATUS.DATA_RDY_INT is not set. Thus,
      --  first read INT_STATUS and continue operation only then DATA_RDY_INT
      --  is set.

      Self.INT_STATUS_Initiate (Success);

      if not Success then
         raise Program_Error;
      end if;
   end On_Interrupt;

   ---------------------------
   -- On_Operation_Finished --
   ---------------------------

   procedure On_Operation_Finished (Self : in out Abstract_MPU_Sensor'Class) is
      use type A0B.I2C.Transfer_State;

      Success : Boolean := True;

   begin
      if Self.Transfer_Status.State /= A0B.I2C.Success then
         Self.State := Initial;

         A0B.Callbacks.Emit_Once (Self.Finished);

         return;
      end if;

      case Self.State is
         when Initial =>
            raise Program_Error;

         when Initialization_WHOAMI_Check =>
            Self.WHOAMI_Check_Complete (Success);
            Self.Device_Reset_Initiate (Success);

         when Initialization_Device_Reset =>
            Self.Device_Reset_Delay_Initiate (Success);

         when Initialization_Device_Reset_Delay =>
            Self.Signal_Path_Reset_Initiate (Success);

         when Initialization_Signal_Path_Reset =>
            Self.Signal_Path_Reset_Delay_Initiate (Success);

         when Initialization_Signal_Path_Reset_Delay =>
            Self.Wakeup_Initiate (Success);

         when Initialization_Wakeup =>
            Self.State := Ready;

            A0B.Callbacks.Emit_Once (Self.Finished);

            return;

         when Configuration_CONFIG =>
            Self.PWR_MGMT_Initiate (Success);

         when Configuration_PWR_MGMT =>
            Self.Configuration_Delay_Initiate (Success);

         when Configuration_Delay =>
            if Self.DMP_Enabled then
               --  Self.Firmware_Upload_Initiate (Success);
               --  Self.Write_DMP (Success);
               raise Program_Error;

            else
               Self.State := Ready;

               A0B.Callbacks.Emit_Once (Self.Finished);

               return;
            end if;

         when Enable_INT_ENABLE_Disable =>
            Self.FIFO_EN_Disable_Initiate (Success);

         when Enable_FIFO_EN_Disable =>
            Self.USER_CTRL_Disable_Initiate (Success);

         when Enable_USER_CTRL_Disable =>
            Self.Reset_Initiate (Success);

         when Enable_Reset =>
            Self.Reset_Delay_Initiate (Success);

         when Enable_Reset_Delay =>
            Self.USER_CTRL_Enable_Initiate (Success);

         when Enable_USER_CTRL_Enable =>
            Self.INT_Enable_Initiate (Success);

         when Enable_INT_Enable =>
            Self.FIFO_EN_Enable_Initiate (Success);

         when Enable_FIFO_EN_Enable =>
            Self.State := Ready;
            Self.INT_Pin.Enable_Interrupt;

            A0B.Callbacks.Emit_Once (Self.Finished);

            return;

         when Interrupt_INT_STATUS =>
            Self.INT_STATUS_Complete_FIFO_COUNT_Initiate (Success);

         when Interrupt_FIFO_COUNT =>
            Self.FIFO_COUNT_Complete_FIFO_R_W_Initiate (Success);

         when Interrupt_FIFO_R_W =>
            Self.FIFO_R_W_Complete (Success);

         when others =>
            raise Program_Error;
      end case;

      if not Success then
         Self.State := Initial;

         raise Program_Error;
      end if;
   end On_Operation_Finished;

   -----------------------
   -- PWR_MGMT_Initiate --
   -----------------------

   procedure PWR_MGMT_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      PWR_MGMT : PWR_MGMT_Registers
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer   : A0B.I2C.Unsigned_8_Array (0 .. 1)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Configuration_PWR_MGMT;

      if Self.DMP_Enabled then
         PWR_MGMT :=
           (PWR_MGMT_1 =>
              (CLKSEL => Registers.Internal,
               SLEEP  => False,
               others => <>),
            PWR_MGMT_2 =>
              (STBY_ZG => False,
               STBY_YG => False,
               STBY_XG => False,
               STBY_ZA => False,
               STBY_YA => False,
               STBY_XA => False,
               others  => <>));
         --  Recommended by Application Notes.

      else
         PWR_MGMT :=
           (PWR_MGMT_1 =>
              (CLKSEL   =>
                   (if Self.Gyroscope_Enabled
                    then Registers.PLL_X
                    else Registers.Internal),
               --  On MPU6500 Auto (PLL_X) can be used always
               TEMP_DIS => not Self.Temperature_Enabled,
               SLEEP    =>
                  not Self.Accelerometer_Enabled
               and not Self.Gyroscope_Enabled
               and not Self.Temperature_Enabled,
               others => <>),
            PWR_MGMT_2 =>
              (STBY_ZG => not Self.Gyroscope_Enabled,
               STBY_YG => not Self.Gyroscope_Enabled,
               STBY_XG => not Self.Gyroscope_Enabled,
               STBY_ZA => not Self.Accelerometer_Enabled,
               STBY_YA => not Self.Accelerometer_Enabled,
               STBY_XA => not Self.Accelerometer_Enabled,
               others  => <>));
      end if;

      Self.Write
        (Address      => PWR_MGMT_1_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end PWR_MGMT_Initiate;

   --------------------------
   -- Reset_Delay_Initiate --
   --------------------------

   procedure Reset_Delay_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      pragma Warnings (Off, Success);

   begin
      if not Success then
         return;
      end if;

      Self.State := Enable_Reset_Delay;

      A0B.Timer.Enqueue
        (Timeout  => Self.Timeout,
         Callback => On_Operation_Finished_Callbacks.Create_Callback (Self),
         T        => A0B.Time.Milliseconds (50));
   end Reset_Delay_Initiate;

   --------------------
   -- Reset_Initiate --
   --------------------

   procedure Reset_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      --  Reset FIFO (and DMP when enabled)

      USER_CTRL :  Registers.USER_CTRL_Register
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer    : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Enable_Reset;

      USER_CTRL :=
        (FIFO_RESET => True,
         DMP_RESET  => Self.DMP_Enabled,
         others     => False);

      Self.Write
        (Address      => USER_CTRL_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end Reset_Initiate;

   --------------------------------------
   -- Signal_Path_Reset_Delay_Initiate --
   --------------------------------------

   procedure Signal_Path_Reset_Delay_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      pragma Warnings (Off, Success);

   begin
      if not Success then
         return;
      end if;

      Self.State := Initialization_Signal_Path_Reset_Delay;

      A0B.Timer.Enqueue
        (Timeout  => Self.Timeout,
         Callback => On_Operation_Finished_Callbacks.Create_Callback (Self),
         T        => A0B.Time.Milliseconds (100));
   end Signal_Path_Reset_Delay_Initiate;

   --------------------------------
   -- Signal_Path_Reset_Initiate --
   --------------------------------

   procedure Signal_Path_Reset_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      SIGNAL_PATH_RESET : Registers.SIGNAL_PATH_RESET_Register
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer            : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Initialization_Signal_Path_Reset;

      SIGNAL_PATH_RESET :=
        (TEMP_Reset  => True,
         ACCEL_Reset => True,
         GYRO_Reset  => True,
         others      => <>);

      Self.Write
        (Address      => SIGNAL_PATH_RESET_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end Signal_Path_Reset_Initiate;

   --  ---------------------
   --  -- Read_DMP_Memory --
   --  ---------------------
   --
   --  procedure Read_DMP_Memory
   --    (Self    : in out Abstract_MPU_Sensor'Class;
   --     Address : Interfaces.Unsigned_16;
   --     Data    : out BBF.Unsigned_8_Array_16;
   --     Success : in out Boolean)
   --  is
   --     use type Interfaces.Unsigned_16;
   --
   --     BANK_SEL   : constant Registers.BANK_SEL_Register :=
   --       (Address => Address);
   --     BANK_SEL_B : constant BBF.Unsigned_8_Array_16 (1 .. DMP_BANK_SEL_Length)
   --       with Import, Address => BANK_SEL'Address;
   --
   --  begin
   --     --  XXX Check that MPU is not in SLEEP state
   --
   --     if Address mod DMP_Bank_Size + Data'Length > DMP_Bank_Size then
   --        --  Prevent write operation to write past the bank boundaries.
   --
   --        Success := False;
   --
   --        return;
   --     end if;
   --
   --     Self.Bus.Write_Synchronous
   --       (Self.Device,
   --        DMP_BANK_SEL_Address,
   --        BANK_SEL_B,
   --        Success);
   --     Self.Bus.Read_Synchronous
   --       (Self.Device,
   --        DMP_MEM_R_W_Address,
   --        Data,
   --        Success);
   --  end Read_DMP_Memory;

   -------------------------
   -- To_Angular_Velosity --
   -------------------------

   function To_Angular_Velosity
     (Self : Abstract_MPU_Sensor'Class;
      Raw  : Interfaces.Integer_16) return Angular_Velosity
   is
      pragma Unreferenced (Self);

      use type Interfaces.Integer_32;

      function Convert is
        new Ada.Unchecked_Conversion
              (Interfaces.Integer_32, Angular_Velosity);

   begin
      return Convert (Interfaces.Integer_32 (Raw) * 1_000 * 8);
      --  XXX 8 must be replaced by configured value
   end To_Angular_Velosity;

   -----------------------------------
   -- To_Gravitational_Acceleration --
   -----------------------------------

   function To_Gravitational_Acceleration
     (Self : Abstract_MPU_Sensor'Class;
      Raw  : Interfaces.Integer_16) return Gravitational_Acceleration
   is
      pragma Unreferenced (Self);

      use type Interfaces.Integer_32;

      function Convert is
        new Ada.Unchecked_Conversion
              (Interfaces.Integer_32, Gravitational_Acceleration);

   begin
      return Convert (Interfaces.Integer_32 (Raw) * 1);
      --  XXX 8 must be replaced by configured value
   end To_Gravitational_Acceleration;

   --  ---------------------
   --  -- Upload_Firmware --
   --  ---------------------
   --
   --  procedure Upload_Firmware
   --    (Self     : in out Abstract_MPU_Sensor'Class;
   --     Firmware : BBF.Unsigned_8_Array_16;
   --     Address  : Interfaces.Unsigned_16;
   --     Success  : in out Boolean)
   --  is
   --     PRGM_START   : constant Registers.PRGM_START_Register :=
   --       (Address => Address);
   --     PRGM_START_B : constant BBF.Unsigned_8_Array_16
   --                               (1 .. DMP_PRGM_START_Length)
   --       with Import, Address => PRGM_START'Address;
   --
   --     Max_Chunk : constant BBF.Unsigned_16 := Self.Buffer'Length;
   --     Current   : BBF.Unsigned_16          := Firmware'First;
   --     Chunk     : BBF.Unsigned_16          := 0;
   --
   --  begin
   --     loop
   --        exit when not Success;
   --        exit when Current > Firmware'Last;
   --
   --        Chunk :=
   --          BBF.Unsigned_16'Min (Max_Chunk, Firmware'Last - Current + 1);
   --
   --        --  Copy chunk of data: when firmware is stored in the flash memory
   --        --  it might be unaccessible by PDC.
   --
   --        Self.Buffer (1 .. Chunk) := Firmware (Current .. Current + Chunk - 1);
   --
   --        Self.Write_DMP_Memory
   --          (Interfaces.Unsigned_16 (Current - 1),
   --           Self.Buffer (1 .. Chunk),
   --           Success);
   --
   --        --  Check that uploaded data match firmware
   --
   --        Self.Read_DMP_Memory
   --          (Interfaces.Unsigned_16 (Current - 1),
   --           Self.Buffer (1 .. Chunk),
   --           Success);
   --
   --        if Firmware (Current .. Current + Chunk - 1)
   --             /= Self.Buffer (1 .. Chunk)
   --        then
   --           Success := False;
   --        end if;
   --
   --        Current := @ + Chunk;
   --     end loop;
   --
   --     Self.Bus.Write_Synchronous
   --       (Self.Device,
   --        DMP_PRGM_START_Address,
   --        PRGM_START_B,
   --        Success);
   --
   --
   --  end Upload_Firmware;
   --
   --  ----------------------
   --  -- Write_DMP_Memory --
   --  ----------------------
   --
   --  procedure Write_DMP_Memory
   --    (Self    : in out Abstract_MPU_Sensor'Class;
   --     Address : Interfaces.Unsigned_16;
   --     Data    : BBF.Unsigned_8_Array_16;
   --     Success : in out Boolean)
   --  is
   --     use type Interfaces.Unsigned_16;
   --
   --     BANK_SEL   : constant Registers.BANK_SEL_Register :=
   --       (Address => Address);
   --     BANK_SEL_B : constant BBF.Unsigned_8_Array_16 (1 .. DMP_BANK_SEL_Length)
   --       with Import, Address => BANK_SEL'Address;
   --
   --  begin
   --     --  XXX Check that MPU is not in SLEEP state
   --
   --     if Address mod DMP_Bank_Size + Data'Length > DMP_Bank_Size then
   --        --  Prevent write operation to write past the bank boundaries.
   --
   --        Success := False;
   --
   --        return;
   --     end if;
   --
   --     Self.Bus.Write_Synchronous
   --       (Self.Device,
   --        DMP_BANK_SEL_Address,
   --        BANK_SEL_B,
   --        Success);
   --     Self.Bus.Write_Synchronous
   --       (Self.Device,
   --        DMP_MEM_R_W_Address,
   --        Data,
   --        Success);
   --  end Write_DMP_Memory;

   --------------------------------
   -- USER_CTRL_Disable_Initiate --
   --------------------------------

   procedure USER_CTRL_Disable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      USER_CTRL : Registers.USER_CTRL_Register
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer    : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Enable_USER_CTRL_Disable;

      USER_CTRL := (others => False);

      Self.Write
        (Address      => USER_CTRL_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end USER_CTRL_Disable_Initiate;

   -------------------------------
   -- USER_CTRL_Enable_Initiate --
   -------------------------------

   procedure USER_CTRL_Enable_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      USER_CTRL : Registers.USER_CTRL_Register
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer    : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Enable_USER_CTRL_Enable;

      USER_CTRL :=
        (FIFO_EN => True,
         DMP_EN  => Self.DMP_Enabled,
         others  => False);

      Self.Write
        (Address      => USER_CTRL_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end USER_CTRL_Enable_Initiate;

   ---------------------
   -- Wakeup_Initiate --
   ---------------------

   procedure Wakeup_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      PWR_MGMT_1 : Registers.PWR_MGMT_1_Register
        with Import, Address => Self.Transfer_Buffer'Address;
      Buffer     : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      Self.State := Initialization_Wakeup;

      PWR_MGMT_1 :=
        (SLEEP  => False,
         CLKSEL => Registers.Internal,
         others => <>);

      Self.Write
        (Address      => PWR_MGMT_1_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end Wakeup_Initiate;

   ---------------------------
   -- WHOAMI_Check_Complete --
   ---------------------------

   procedure WHOAMI_Check_Complete
     (Self    : in out Abstract_MPU_Sensor'Class;
      Success : in out Boolean)
   is
      use type A0B.Types.Unsigned_8;

   begin
      if not Success then
         return;
      end if;

      if Self.Transfer_Buffer (0) /= Self.WHOAMI then
         Success := False;
      end if;
   end WHOAMI_Check_Complete;

   ---------------------------
   -- WHOAMI_Check_Initiate --
   ---------------------------

   procedure WHOAMI_Check_Initiate
     (Self    : in out Abstract_MPU_Sensor'Class;
      WHOAMI  : A0B.Types.Unsigned_8;
      Success : in out Boolean)
   is
      Buffer : A0B.I2C.Unsigned_8_Array (0 .. 0)
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success then
         return;
      end if;

      --  Initiate read of controller's WHOAMI code

      Self.State  := Initialization_WHOAMI_Check;
      Self.WHOAMI := WHOAMI;

      Self.Read
        (Address      => A0B.MPUXXXX.WHO_AM_I_Address,
         Buffer       => Buffer,
         Status       => Self.Transfer_Status,
         On_Completed => On_Operation_Finished_Callbacks.Create_Callback (Self),
         Success      => Success);
   end WHOAMI_Check_Initiate;

end A0B.MPUXXXX;
