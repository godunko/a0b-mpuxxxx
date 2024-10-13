--
--  Copyright (C) 2019-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

pragma Restrictions (No_Elaboration_Code);

with Ada.Unchecked_Conversion;

package body A0B.MPUXXXX.MPU6500.MPU9250 is

   ---------
   -- Get --
   ---------

   procedure Get
     (Data           : Calibration_Data;
      Acceleration_X : out Gravitational_Acceleration;
      Acceleration_Y : out Gravitational_Acceleration;
      Acceleration_Z : out Gravitational_Acceleration;
      Velocity_U     : out Angular_Velosity;
      Velocity_V     : out Angular_Velosity;
      Velocity_W     : out Angular_Velosity)
   is
      use type A0B.Types.Integer_32;

      function To_AV is
        new Ada.Unchecked_Conversion
              (A0B.Types.Integer_32, Angular_Velosity);

      function To_GA is
        new Ada.Unchecked_Conversion
              (A0B.Types.Integer_32, Gravitational_Acceleration);

      function Calibration_To_AV
        (Item : A0B.Types.Integer_16) return Angular_Velosity;

      function Calibration_To_GA
        (Item : A0B.Types.Integer_16) return Gravitational_Acceleration;

      -----------------------
      -- Calibration_To_AV --
      -----------------------

      function Calibration_To_AV
        (Item : A0B.Types.Integer_16) return Angular_Velosity is
      begin
         return To_AV (A0B.Types.Integer_32 (Item) * 1_000 * 4);
      end Calibration_To_AV;

      -----------------------
      -- Calibration_To_GA --
      -----------------------

      function Calibration_To_GA
        (Item : A0B.Types.Integer_16) return Gravitational_Acceleration is
      begin
         return To_GA (A0B.Types.Integer_32 (Item) * 8);
      end Calibration_To_GA;

   begin
      Acceleration_X := Calibration_To_GA (Data.Accelerometer.X_OFFS_USR);
      Acceleration_Y := Calibration_To_GA (Data.Accelerometer.Y_OFFS_USR);
      Acceleration_Z := Calibration_To_GA (Data.Accelerometer.Z_OFFS_USR);

      Velocity_U := Calibration_To_AV (Data.Gyroscope.X_OFFS_USR);
      Velocity_V := Calibration_To_AV (Data.Gyroscope.Y_OFFS_USR);
      Velocity_W := Calibration_To_AV (Data.Gyroscope.Z_OFFS_USR);
   end Get;

   ---------
   -- Get --
   ---------

   procedure Get
     (Self      : MPU9250_Sensor'Class;
      Data      : out Sensor_Data;
      Timestamp : out A0B.Time.Monotonic_Time)
   is
      Raw : Raw_Data renames Self.Raw_Data (Self.User_Bank);

   begin
      if Self.Accelerometer_Enabled then
         Data.Acceleration_X :=
           Self.To_Gravitational_Acceleration (Raw.ACCEL.XOUT);
         Data.Acceleration_Y :=
           Self.To_Gravitational_Acceleration (Raw.ACCEL.YOUT);
         Data.Acceleration_Z :=
           Self.To_Gravitational_Acceleration (Raw.ACCEL.ZOUT);

      else
         Data.Acceleration_X := 0.0;
         Data.Acceleration_Y := 0.0;
         Data.Acceleration_Z := 0.0;
      end if;

      if Self.Temperature_Enabled then
         Data.Temperature :=
           Self.To_Temperature (Raw.TEMP.TEMP_OUT);

      else
         Data.Temperature := 0.0;
      end if;

      if Self.Gyroscope_Enabled then
         Data.Velocity_U := Self.To_Angular_Velosity (Raw.GYRO.XOUT);
         Data.Velocity_V := Self.To_Angular_Velosity (Raw.GYRO.YOUT);
         Data.Velocity_W := Self.To_Angular_Velosity (Raw.GYRO.ZOUT);

      else
         Data.Velocity_U := 0.0;
         Data.Velocity_V := 0.0;
         Data.Velocity_W := 0.0;
      end if;

      if Self.Accelerometer_Enabled
        or Self.Temperature_Enabled
        or Self.Gyroscope_Enabled
      then
         Timestamp := Raw.Timestamp;

      else
         Timestamp := A0B.Time.To_Monotonic_Time (0);
      end if;
   end Get;

   ---------------------
   -- Get_Calibration --
   ---------------------

   procedure Get_Calibration
     (Self     : in out MPU9250_Sensor'Class;
      Data     : out Calibration_Data;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean) is
   begin
      if not Success or Self.State /= Ready then
         Success := False;

         return;
      end if;

      Self.Calibration_Buffer :=
        A0B.MPUXXXX.Calibration_Data (Data)'Unrestricted_Access;
      Self.Finished           := Finished;

      Self.Get_XA_OFFS_USR_Initiate (Success);
   end Get_Calibration;

   ----------------
   -- Initialize --
   ----------------

   overriding procedure Initialize
     (Self     : in out MPU9250_Sensor;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean) is
   begin
      Self.Internal_Initialize (MPU9250_WHOAMI, Finished, Success);

      --  XXX Initialize compass
   end Initialize;

   ---------
   -- Set --
   ---------

   procedure Set
     (Data           : out Calibration_Data;
      Acceleration_X : Gravitational_Acceleration;
      Acceleration_Y : Gravitational_Acceleration;
      Acceleration_Z : Gravitational_Acceleration;
      Velocity_U     : Angular_Velosity;
      Velocity_V     : Angular_Velosity;
      Velocity_W     : Angular_Velosity)
   is
      use type A0B.Types.Integer_32;

      function To_I32 is
        new Ada.Unchecked_Conversion
              (Angular_Velosity, A0B.Types.Integer_32);

      function To_I32 is
        new Ada.Unchecked_Conversion
              (Gravitational_Acceleration, A0B.Types.Integer_32);

      function Calibration_To_I16
        (Item : Angular_Velosity) return A0B.Types.Integer_16;

      function Calibration_To_I16
        (Item : Gravitational_Acceleration) return A0B.Types.Integer_16;

      ------------------------
      -- Calibration_To_I16 --
      ------------------------

      function Calibration_To_I16
        (Item : Angular_Velosity) return A0B.Types.Integer_16 is
      begin
         return A0B.Types.Integer_16 (To_I32 (Item) / 1_000 / 4);
      end Calibration_To_I16;

      ------------------------
      -- Calibration_To_I16 --
      ------------------------

      function Calibration_To_I16
        (Item : Gravitational_Acceleration) return A0B.Types.Integer_16 is
      begin
         return A0B.Types.Integer_16 (To_I32 (Item) / 8);
      end Calibration_To_I16;

   begin
      Data.Accelerometer.X_OFFS_USR := Calibration_To_I16 (Acceleration_X);
      Data.Accelerometer.Y_OFFS_USR := Calibration_To_I16 (Acceleration_Y);
      Data.Accelerometer.Z_OFFS_USR := Calibration_To_I16 (Acceleration_Z);

      Data.Gyroscope.X_OFFS_USR := Calibration_To_I16 (Velocity_U);
      Data.Gyroscope.Y_OFFS_USR := Calibration_To_I16 (Velocity_V);
      Data.Gyroscope.Z_OFFS_USR := Calibration_To_I16 (Velocity_W);
   end Set;

   ---------------------
   -- Set_Calibration --
   ---------------------

   procedure Set_Calibration
     (Self     : in out MPU9250_Sensor'Class;
      Data     : Calibration_Data;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean)
   is
      Buffer : Calibration_Data
        with Import, Address => Self.Transfer_Buffer'Address;

   begin
      if not Success or Self.State /= Ready then
         Success := False;

         return;
      end if;

      Buffer        := Data;
      Self.Finished := Finished;

      Self.Set_XA_OFFS_USR_Initiate (Success);
   end Set_Calibration;

end A0B.MPUXXXX.MPU6500.MPU9250;
