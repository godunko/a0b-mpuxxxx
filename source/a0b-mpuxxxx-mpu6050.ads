--
--  Copyright (C) 2019-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--   MPU-6050: The Motion Processing Unit

pragma Restrictions (No_Elaboration_Code);

with A0B.Time;

package A0B.MPUXXXX.MPU6050 is

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

   type MPU6050_Sensor is
     new Abstract_MPU_Sensor with private with Preelaborable_Initialization;

   not overriding procedure Initialize
     (Self     : in out MPU6050_Sensor;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean);

   procedure Get
     (Self      : MPU6050_Sensor'Class;
      Data      : out Sensor_Data;
      Timestamp : out A0B.Time.Monotonic_Time);

private

   type MPU6050_Sensor is new Abstract_MPU_Sensor with null record;

   overriding function Is_6500_9250
     (Self : MPU6050_Sensor) return Boolean is (False);

   overriding function To_Temperature
     (Self : MPU6050_Sensor;
      Raw  : Interfaces.Integer_16) return Temperature;

end A0B.MPUXXXX.MPU6050;
