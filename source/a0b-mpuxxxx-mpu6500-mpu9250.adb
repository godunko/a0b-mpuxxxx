--
--  Copyright (C) 2019-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

pragma Restrictions (No_Elaboration_Code);

package body A0B.MPUXXXX.MPU6500.MPU9250 is

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

   ----------------
   -- Initialize --
   ----------------

   procedure Initialize
     (Self     : in out MPU9250_Sensor;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean) is
   begin
      Self.Internal_Initialize (MPU9250_WHOAMI, Finished, Success);

      --  XXX Initialize compass
   end Initialize;

end A0B.MPUXXXX.MPU6500.MPU9250;
