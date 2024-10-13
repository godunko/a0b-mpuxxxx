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

   type Calibration_Data is private;

   procedure Get
     (Data           : Calibration_Data;
      Acceleration_X : out Gravitational_Acceleration;
      Acceleration_Y : out Gravitational_Acceleration;
      Acceleration_Z : out Gravitational_Acceleration;
      Velocity_U     : out Angular_Velosity;
      Velocity_V     : out Angular_Velosity;
      Velocity_W     : out Angular_Velosity);

   procedure Set
     (Data           : out Calibration_Data;
      Acceleration_X : Gravitational_Acceleration;
      Acceleration_Y : Gravitational_Acceleration;
      Acceleration_Z : Gravitational_Acceleration;
      Velocity_U     : Angular_Velosity;
      Velocity_V     : Angular_Velosity;
      Velocity_W     : Angular_Velosity);

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

   procedure Get_Calibration
     (Self     : in out MPU9250_Sensor'Class;
      Data     : out Calibration_Data;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean);

   procedure Set_Calibration
     (Self     : in out MPU9250_Sensor'Class;
      Data     : Calibration_Data;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean);

private

   type Calibration_Data is new A0B.MPUXXXX.Calibration_Data;

   type MPU9250_Sensor is new MPU6500_Sensor with null record;

end A0B.MPUXXXX.MPU6500.MPU9250;
