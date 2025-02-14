#
# Matter_Plugin_Sensor_OnOff.be - implements the behavior for a Occupany Switch
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

# Matter plug-in for core behavior

# dummy declaration for solidification
class Matter_Plugin_Device end

#@ solidify:Matter_Plugin_Sensor_OnOff,weak

class Matter_Plugin_Sensor_OnOff : Matter_Plugin_Device
  static var TYPE = "occupancy"                     # name of the plug-in in json
  static var NAME = "OnOff"                         # display name of the plug-in
  static var ARG  = "switch"                        # additional argument name (or empty if none)
  static var ARG_TYPE = / x -> int(x)               # function to convert argument to the right type
  static var UPDATE_TIME = 5000                     # update every 250ms
  static var CLUSTERS  = {
    0x0006: [0,0xFFFC,0xFFFD],                      # On/Off 1.5 p.48
  }
  static var TYPES = { 0x0850: 2 }                  # OnOff Sensor, rev 2

  var tasmota_switch_index                          # Switch number in Tasmota (one based)
  var shadow_onoff

  #############################################################
  # Constructor
  def init(device, endpoint, arguments)
    super(self).init(device, endpoint, arguments)
    self.tasmota_switch_index = int(arguments.find(self.ARG #-'relay'-#, 1))
    if self.tasmota_switch_index <= 0    self.tasmota_switch_index = 1    end
  end

  #############################################################
  # Update shadow
  #
  def update_shadow()
    super(self).update_shadow()

    import json
    var ret = tasmota.cmd("Status 8", true)
    if ret != nil
      var j = json.load(ret)
      if j != nil
        var state = false
        state = (j.find("Switch" + str(self.tasmota_switch_index)) == "ON")

        if self.shadow_onoff != nil && self.shadow_onoff != bool(state)
          self.attribute_updated(0x0406, 0x0000)
        end
        self.shadow_onoff = state
      end
    end
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
    if   cluster == 0x0006              # ========== On/Off 1.5 p.48 ==========
      self.update_shadow_lazy()
      if   attribute == 0x0000          #  ---------- OnOff / bool ----------
        return TLV.create_TLV(TLV.BOOL, self.shadow_onoff)
      elif attribute == 0xFFFC          #  ---------- FeatureMap / map32 ----------
        return TLV.create_TLV(TLV.U4, 0)    # 0 = no Level Control for Lighting
      elif attribute == 0xFFFD          #  ---------- ClusterRevision / u2 ----------
        return TLV.create_TLV(TLV.U4, 4)    # 0 = no Level Control for Lighting
      end

    else
      return super(self).read_attribute(session, ctx)
    end
  end

end
matter.Plugin_Sensor_OnOff = Matter_Plugin_Sensor_OnOff
