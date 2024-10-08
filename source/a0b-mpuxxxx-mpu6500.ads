--
--  Copyright (C) 2019-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--   MPU-6500: The Motion Processing Unit

pragma Restrictions (No_Elaboration_Code);

package A0B.MPUXXXX.MPU6500 is

   pragma Preelaborate;

   type MPU6500_Sensor is new Abstract_MPU_Sensor with private;

   not overriding procedure Initialize
     (Self     : in out MPU6500_Sensor;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean);

private

   type MPU6500_Sensor is new Abstract_MPU_Sensor with null record;

   overriding function Is_6500_9250
     (Self : MPU6500_Sensor) return Boolean is (True);

   overriding function To_Temperature
     (Self : MPU6500_Sensor;
      Raw  : Interfaces.Integer_16) return Temperature;

end A0B.MPUXXXX.MPU6500;
