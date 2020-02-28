// Copyright 2020 Rensselaer Polytechnic Institute
//                Wason Technology, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using System.Text;
using com.robotraconteur.device;
using com.robotraconteur.uuid;
using com.robotraconteur.geometry;
using System.IO;
using System.Linq;

namespace RobotRaconteurWeb.InfoParser
{
    public class YamlDeviceInfo
    {
        public YamlIdentifier device { get; set; }
        public YamlIdentifier parent_device { get; set; }
        public YamlIdentifier manufacturer { get; set; }
        public YamlIdentifier model { get; set; }
        public List<YamlDeviceOption> options { get; set; }
        public List<YamlDeviceCapability> capabilities { get; set; }
        public string serial_number { get; set; }
        public List<YamlDeviceClass> device_classes;
        public string user_description { get; set; }
        public YamlResourceIdentifier description_resource { get; set; }
        public List<string> implemented_types { get; set; }
        public YamlNamedPose device_origin_pose { get; set; }
        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(com.robotraconteur.device.DeviceInfo device_info)
        {
            device_info.device = device?.ToRRInfo();
            device_info.parent_device = parent_device?.ToRRInfo();
            device_info.manufacturer = manufacturer?.ToRRInfo();
            device_info.model = model?.ToRRInfo();
            device_info.serial_number = serial_number ?? "";
            device_info.options = options?.Select(x => x.ToRRInfo()).ToList();
            device_info.capabilities = capabilities?.Select(x => x.ToRRInfo()).ToList();
            device_info.device_classes = device_classes?.Select(x => x.ToRRInfo()).ToList();
            device_info.user_description = user_description ?? "";
            device_info.description_resource = description_resource?.ToRRInfo();
            device_info.implemented_types = implemented_types?.Select(x => x ?? "").ToList();
            device_info.device_origin_pose = device_origin_pose?.ToRRInfo();
            device_info.extended = extended?.ToDictionary(x => x.Key, x => x.Value.value);
        }

        public com.robotraconteur.device.DeviceInfo ToRRInfo()
        {
            var info = new com.robotraconteur.device.DeviceInfo();
            CopyTo(info);
            return info;
        }
    }

    public class YamlDeviceSubOption
    {
        public string suboption_name { get; set; }
        public double suboption_level { get; set; }

        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(DeviceSubOption device_sub_option)
        {
            device_sub_option.suboption_name = suboption_name ?? "";
            device_sub_option.suboption_level = suboption_level;
            device_sub_option.extended = extended?.ToDictionary(x => x.Key, x => x.Value.value);
        }

        public DeviceSubOption ToRRInfo()
        {
            var info = new DeviceSubOption();
            CopyTo(info);
            return info;
        }
    }
    public class YamlDeviceOption
    {
        public YamlIdentifier option_identifier { get; set; }

        public List<YamlDeviceSubOption> suboptions { get; set; }

        public void CopyTo(DeviceOption device_option)
        {
            device_option.option_identifier = option_identifier?.ToRRInfo();
            device_option.suboptions = suboptions?.Select(x => x?.ToRRInfo()).ToList();
        }

        public DeviceOption ToRRInfo()
        {
            var info = new DeviceOption();
            CopyTo(info);
            return info;
        }
    }

    public class YamlDeviceSubCapability
    {
        public string subcapability_name { get; set; }
        public double subcapability_level { get; set; }

        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(DeviceSubCapability device_sub_option)
        {
            device_sub_option.subcapability_name = subcapability_name ?? "";
            device_sub_option.subcapability_level = subcapability_level;
            device_sub_option.extended = extended?.ToDictionary(x => x.Key, x => x.Value.value);
        }

        public DeviceSubCapability ToRRInfo()
        {
            var info = new DeviceSubCapability();
            CopyTo(info);
            return info;
        }
    }

    public class YamlDeviceCapability
    {
        public YamlIdentifier capability_identifier { get; set; }

        public List<YamlDeviceSubCapability> subcapabilities { get; set; }

        public void CopyTo(DeviceCapability device_option)
        {
            device_option.capability_identifier = capability_identifier?.ToRRInfo();
            device_option.subcapabilities = subcapabilities?.Select(x => x?.ToRRInfo()).ToList();
        }

        public DeviceCapability ToRRInfo()
        {
            var info = new DeviceCapability();
            CopyTo(info);
            return info;
        }
    }

    public class YamlDeviceClass
    {
        public YamlIdentifier class_identifier { get; set; }
        public List<string> subclasses { get; set; }

        public void CopyTo(com.robotraconteur.device.DeviceClass c)
        {
            c.class_identifier = class_identifier?.ToRRInfo();
            c.subclasses = subclasses?.Select(x => x ?? "").ToList();
        }

        public com.robotraconteur.device.DeviceClass ToRRInfo()
        {
            var c = new com.robotraconteur.device.DeviceClass();
            CopyTo(c);
            return c;
        }
    }

    public static class DeviceInfoParser
    {
        public static Tuple<DeviceInfo, LocalIdentifierLocks> LoadDeviceInfoYamlWithIdentifierLocks(string filename)
        {
            using (var f = new StreamReader(filename))
            {
                var deserializer = new YamlDotNet.Serialization.Deserializer();
                var yaml_device_info = deserializer.Deserialize<YamlDeviceInfo>(f);
                return LoadDeviceInfoYamlWithIdentifierLocks(yaml_device_info);
            }
        }

        public static Tuple<DeviceInfo, LocalIdentifierLocks> ParseDeviceInfoYamlWithIdentifierLocks(string data)
        {
            using (var f = new StringReader(data))
            {
                var deserializer = new YamlDotNet.Serialization.Deserializer();
                var yaml_device_info = deserializer.Deserialize<YamlDeviceInfo>(f);
                return LoadDeviceInfoYamlWithIdentifierLocks(yaml_device_info);
            }
        }

        public static Tuple<DeviceInfo, LocalIdentifierLocks> LoadDeviceInfoYamlWithIdentifierLocks(YamlDeviceInfo yaml_device_info)
        {
            throw new NotImplementedException();
        }        
    }

}