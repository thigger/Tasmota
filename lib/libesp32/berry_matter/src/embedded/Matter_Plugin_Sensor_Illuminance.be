#
# Matter_Plugin_Sensor_Light.be - implements the behavior for a Light Sensor
#
# Copyright (C) 2023  Stephan Hadinger & Theo Arends
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import matter

# Matter plug-in for core behavior

# dummy declaration for solidification
class Matter_Plugin_Sensor end

#@ solidify:Matter_Plugin_Sensor_Illuminance,weak

class Matter_Plugin_Sensor_Illuminance : Matter_Plugin_Sensor
  static var TYPE = "illuminance"                   # name of the plug-in in json
  static var NAME = "Illuminance"                   # display name of the plug-in
  static var CLUSTERS  = {
    0x0400: [0,1,2,0xFFFC,0xFFFD],                  # Illuminance Measurement p.95 - no writable
  }
  static var TYPES = { 0x0106: 2 }                  # Illuminance Sensor, rev 2

  #############################################################
  # Pre-process value
  #
  # This must be overriden.
  # This allows to convert the raw sensor value to the target one, typically int
  def pre_value(val)
    return val != nil ? int(val) : nil        # value in lux
  end

  #############################################################
  # Called when the value changed compared to shadow value
  #
  # This must be overriden.
  # This is where you call `self.attribute_updated(<cluster>, <attribute>)`
  def value_changed(val)
    self.attribute_updated(0x0400, 0x0000)
  end

  #############################################################
  # read an attribute
  #
  def read_attribute(session, ctx)
    import string
    var TLV = matter.TLV
    var cluster = ctx.cluster
    var attribute = ctx.attribute

    # ====================================================================================================
    if   cluster == 0x0400              # ========== Illuminance Measurement 2.2 p.95 ==========
      if   attribute == 0x0000          #  ---------- MeasuredValue / i16 ----------
        if self.shadow_value != nil
          return TLV.create_TLV(TLV.I2, int(self.shadow_value))
        else
          return TLV.create_TLV(TLV.NULL, nil)
        end
      elif attribute == 0x0001          #  ---------- MinMeasuredValue / i16 ----------
        return TLV.create_TLV(TLV.I2, 0)  # 0 lux
      elif attribute == 0x0002          #  ---------- MaxMeasuredValue / i16 ----------
        return TLV.create_TLV(TLV.I2, 10000)  # 10000 lux
      elif attribute == 0xFFFC          #  ---------- FeatureMap / map32 ----------
        return TLV.create_TLV(TLV.U4, 0)
      elif attribute == 0xFFFD          #  ---------- ClusterRevision / u2 ----------
        return TLV.create_TLV(TLV.U4, 3)    # 3 = New data model format and notation
      end

    else
      return super(self).read_attribute(session, ctx)
    end
  end

end
matter.Plugin_Sensor_Illuminance = Matter_Plugin_Sensor_Illuminance
