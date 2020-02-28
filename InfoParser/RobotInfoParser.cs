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
using com.robotraconteur.robotics.robot;
using com.robotraconteur.uuid;
using com.robotraconteur.geometry;
using System.IO;
using System.Linq;

namespace RobotRaconteurWeb.InfoParser
{
    public class YamlRobotInfo
    {
        public YamlDeviceInfo device_info { get; set; }
        public com.robotraconteur.robotics.robot.RobotType robot_type { get; set; }
        public List<YamlJointInfo> joint_info { get; set; }
        public List<YamlRobotKinChainInfo> chains { get; set; }
        public List<com.robotraconteur.robotics.robot.RobotCapabilities> robot_capabilities { get; set; }
        public List<YamlSignalInfo> signal_info { get; set; }
        public List<YamlParameterInfo> parameter_info { get; set; }

        public Dictionary<string,YamlVarValue> extended { get; set; }

        public void CopyTo(com.robotraconteur.robotics.robot.RobotInfo robot_info)
        {
            robot_info.device_info = device_info?.ToRRInfo();
            robot_info.robot_type = robot_type;
            robot_info.joint_info = joint_info?.Select(x => x?.ToRRInfo()).ToList();
            robot_info.chains = chains?.Select(x => x?.ToRRInfo()).ToList();
            robot_info.robot_capabilities = (uint)(robot_capabilities?.Aggregate((x, y) => x | y) ?? 0);
            robot_info.signal_info = signal_info?.Select(x => x?.ToRRInfo()).ToList();
            robot_info.parameter_info = parameter_info?.Select(x => x?.ToRRInfo()).ToList();
            robot_info.extended = extended?.ToDictionary(x => x.Key, x => x.Value?.value);
        }

        public com.robotraconteur.robotics.robot.RobotInfo ToRRInfo()
        {
            var info = new com.robotraconteur.robotics.robot.RobotInfo();
            CopyTo(info);
            return info;
        }
    }

    public class YamlJointInfo
    {
        public YamlIdentifier joint_identifier { get; set; }
        public com.robotraconteur.robotics.joints.JointType joint_type { get; set; }
        public YamlJointLimits joint_limits { get; set; }
        public com.robotraconteur.robotics.joints.JointPositionUnits default_units { get; set; }
        public com.robotraconteur.robotics.joints.JointEffortUnits default_effort_units { get; set; }
        public bool passive { get; set; }

        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(com.robotraconteur.robotics.joints.JointInfo joint_info)
        {
            joint_info.joint_identifier = joint_identifier?.ToRRInfo();
            joint_info.joint_type = joint_type;
            joint_info.joint_limits = joint_limits?.ToRRInfo();
            joint_info.default_units = default_units;
            joint_info.default_effort_units = default_effort_units;
            joint_info.passive = passive;
            joint_info.extended = extended?.ToDictionary(x => x.Key, x => x.Value?.value);
        }

        public com.robotraconteur.robotics.joints.JointInfo ToRRInfo()
        {
            var info = new com.robotraconteur.robotics.joints.JointInfo();
            CopyTo(info);
            return info;
        }
    }

    public class YamlRobotKinChainInfo
    {
        public YamlIdentifier kin_chain_identifier { get; set; }
        public List<YamlVector3> H { get; set; }
        public List<YamlVector3> P { get; set; }
        public List<YamlSpatialInertia> link_inertias { get; set; }
        public List<YamlIdentifier> link_identifiers { get; set; }
        public List<uint> joint_numbers { get; set; }
        public YamlPose flange_pose { get; set; }
        public YamlIdentifier flange_identifier { get; set; }
        public YamlToolInfo current_tool { get; set; }
        public YamlPayloadInfo current_payload { get; set; }
        public YamlSpatialVelocity tcp_max_velocity { get; set; }
        public YamlSpatialAcceleration tcp_max_acceleration { get; set; }

        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(com.robotraconteur.robotics.robot.RobotKinChainInfo info)
        {
            info.kin_chain_identifier = kin_chain_identifier?.ToRRInfo();
            info.H = H?.Select(x => x.ToRRInfo()).ToArray() ?? new Vector3[0];
            info.P = P?.Select(x => x.ToRRInfo()).ToArray() ?? new Vector3[0];
            info.link_inertias = link_inertias?.Select(x => x.ToRRInfo()).ToArray() ?? new SpatialInertia[0];
            info.link_identifiers = link_identifiers?.Select(x => x.ToRRInfo()).ToList();
            info.joint_numbers = joint_numbers?.ToArray() ?? new uint[0];
            info.flange_pose = flange_pose?.ToRRInfo() ?? new Pose();
            info.flange_identifier = flange_identifier?.ToRRInfo();
            info.current_tool = current_tool?.ToRRInfo();
            info.current_payload = current_payload?.ToRRInfo();
            info.tcp_max_velocity = tcp_max_velocity?.ToRRInfo() ?? default;
            info.tcp_max_acceleration = tcp_max_acceleration?.ToRRInfo() ?? default;
            info.extended = extended?.ToDictionary(x => x.Key, x => x.Value?.value);
        }

        public com.robotraconteur.robotics.robot.RobotKinChainInfo ToRRInfo()
        {
            var info = new com.robotraconteur.robotics.robot.RobotKinChainInfo();
            CopyTo(info);
            return info;
        }
    }

    public class YamlToolInfo
    {
        public YamlDeviceInfo device_info { get; set; }
        public YamlTransform tcp { get; set; }

        public YamlSpatialInertia inertia { get; set; }

        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(com.robotraconteur.robotics.tool.ToolInfo info)
        {
            info.device_info = device_info?.ToRRInfo();
            info.tcp = tcp?.ToRRInfo() ?? new Transform();
            info.inertia = inertia?.ToRRInfo() ?? new SpatialInertia();
            info.extended = extended?.ToDictionary(x => x.Key, x => x.Value?.value);
        }

        public com.robotraconteur.robotics.tool.ToolInfo ToRRInfo()
        {
            var info = new com.robotraconteur.robotics.tool.ToolInfo();
            CopyTo(info);
            return info;
        }
    }

    public class YamlPayloadInfo
    {
        public YamlDeviceInfo device_info { get; set; }
        public YamlSpatialInertia inertia { get; set; }

        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(com.robotraconteur.robotics.payload.PayloadInfo info)
        {
            info.device_info = device_info?.ToRRInfo();
            info.inertia = inertia?.ToRRInfo() ?? new SpatialInertia();
            info.extended = extended?.ToDictionary(x => x.Key, x => x.Value?.value);
        }

        public com.robotraconteur.robotics.payload.PayloadInfo ToRRInfo()
        {
            var info = new com.robotraconteur.robotics.payload.PayloadInfo();
            CopyTo(info);
            return info;
        }
    }


    public class YamlJointLimits
    {
        public double lower { get; set; }
        public double upper { get; set; }
        public double velocity { get; set; }
        public double acceleration { get; set; }
        public double effort { get; set; }

        public void CopyTo(com.robotraconteur.robotics.joints.JointLimits info)
        {
            info.lower = lower;
            info.upper = upper;
            info.velocity = velocity;
            info.acceleration = acceleration;
            info.effort = effort;
        }

        public com.robotraconteur.robotics.joints.JointLimits ToRRInfo()
        {
            var info = new com.robotraconteur.robotics.joints.JointLimits();
            CopyTo(info);
            return info;
        }
    }

    public class YamlSignalInfo
    {
        public YamlIdentifier signal_identifier { get; set; }
        public YamlDeviceClass signal_class { get; set; }
        //TODO: public List<YamlSIUnit> units { get; set; }
        public YamlDataType data_type { get; set; }
        public com.robotraconteur.signal.SignalType signal_type { get; set; }
        public com.robotraconteur.signal.SignalAccessLevel access_level { get; set; }
        public List<uint> address { get; set; }
        public string user_description { get; set; }
        public double max_update_rate { get; set; }
        public double[] min_value { get; set; }
        public double[] max_value { get; set; }

        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(com.robotraconteur.signal.SignalInfo info)
        {
            info.signal_identifier = signal_identifier?.ToRRInfo();
            info.signal_class = signal_class?.ToRRInfo();
            info.data_type = data_type?.ToRRInfo();
            info.signal_type = signal_type;
            info.access_level = access_level;
            info.address = address?.ToArray() ?? new uint[0];
            info.user_description = user_description ?? "";
            info.max_update_rate = max_update_rate;
            info.min_value = min_value ?? new double[0];
            info.max_value = max_value ?? new double[0];
            info.extended = extended?.ToDictionary(x => x.Key, x => x.Value?.value);
        }

        public com.robotraconteur.signal.SignalInfo ToRRInfo()
        {
            var info = new com.robotraconteur.signal.SignalInfo();
            CopyTo(info);
            return info;
        }
    }

    public static class RobotInfoParser
    {
        public static Tuple<RobotInfo, LocalIdentifierLocks> LoadRobotInfoYamlWithIdentifierLocks(string filename)
        {
            using (var f = new StreamReader(filename))
            {
                var deserializer = new YamlDotNet.Serialization.Deserializer();
                var yaml_robot_info = deserializer.Deserialize<YamlRobotInfo>(f);
                return LoadRobotInfoYamlWithIdentifierLocks(yaml_robot_info);
            }
        }

        public static Tuple<RobotInfo, LocalIdentifierLocks> ParseRobotInfoYamlWithIdentifierLocks(string data)
        {
            using (var f = new StringReader(data))
            {
                var deserializer = new YamlDotNet.Serialization.Deserializer();
                var yaml_robot_info = deserializer.Deserialize<YamlRobotInfo>(f);
                return LoadRobotInfoYamlWithIdentifierLocks(yaml_robot_info);
            }
        }

        
        public static Tuple<RobotInfo, LocalIdentifierLocks> LoadRobotInfoYamlWithIdentifierLocks(YamlRobotInfo yaml_robot_info)
        {
            var robot_info = yaml_robot_info.ToRRInfo();
            LocalIdentifierLocks ident_locks = null;
            if (robot_info.device_info != null && robot_info.device_info.device != null 
                && !String.IsNullOrEmpty(robot_info.device_info.device.name)
                && robot_info.device_info.device.uuid.uuid_bytes.All(x=> x ==0))
            {
                var l = LocalIdentifiersManager.GetIdentifierForNameAndLock("robot", robot_info.device_info.device.name);
                robot_info.device_info.device.uuid = l.Identifier.uuid;
                ident_locks = new LocalIdentifierLocks(l);                
            }            
            return Tuple.Create<RobotInfo,LocalIdentifierLocks>(robot_info,ident_locks);
        }
    }
}
