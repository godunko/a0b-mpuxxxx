--
--  Copyright (C) 2019-2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

pragma Restrictions (No_Elaboration_Code);

package body A0B.MPUXXXX.MPU6500 is

   ----------------
   -- Initialize --
   ----------------

   not overriding procedure Initialize
     (Self     : in out MPU6500_Sensor;
      Finished : A0B.Callbacks.Callback;
      Success  : in out Boolean) is
   begin
      Self.Internal_Initialize (MPU6500_WHOAMI, Finished, Success);
   end Initialize;

   --------------------
   -- To_Temperature --
   --------------------

   overriding function To_Temperature
     (Self : MPU6500_Sensor;
      Raw  : Interfaces.Integer_16) return Temperature is
   begin
      return Temperature (Float (Raw) / 321.0 + 21.0);
   end To_Temperature;

end A0B.MPUXXXX.MPU6500;
