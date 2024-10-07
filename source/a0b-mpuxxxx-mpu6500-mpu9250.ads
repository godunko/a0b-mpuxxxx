--
--  Copyright (C) 2019-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--   MPU-9250: The Motion Processing Unit

pragma Restrictions (No_Elaboration_Code);

with A0B.Time;

package A0B.MPUXXXX.MPU6500.MPU9250 is

   pragma Preelaborate;

   type Sensor_Data is record
      Acceleration_X : Gravitational_Acceleration;
      Acceleration_Y : Gravitational_Acceleration;
      Acceleration_Z : Gravitational_Acceleration;
      Velocity_U     : Angular_Velosity;
      Velocity_V     : Angular_Velosity;
      Velocity_W     : Angular_Velosity;
      Temperature    : A0B.MPUXXXX.Temperature;
   end record;

   type MPU9250_Sensor is
     new Abstract_MPU_Sensor with private with Preelaborable_Initialization;

   procedure Initialize
     (Self     : in out MPU9250_Sensor;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean);

   procedure Get
     (Self      : MPU9250_Sensor'Class;
      Data      : out Sensor_Data;
      Timestamp : out A0B.Time.Monotonic_Time);

private

   type MPU9250_Sensor is new MPU6500_Sensor with null record;

   overriding function To_Temperature
     (Self : MPU9250_Sensor;
      Raw  : Interfaces.Integer_16) return Temperature is
        (0.0);

end A0B.MPUXXXX.MPU6500.MPU9250;
