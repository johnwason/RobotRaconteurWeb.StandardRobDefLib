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

namespace RobotRaconteurWeb.InfoParser
{

    public class YamlNamedPose
    {
        public YamlIdentifier parent_frame { get; set; }
        public YamlIdentifier frame { get; set; }
        
        public YamlPose pose { get; set; }

        public void CopyTo(com.robotraconteur.geometry.NamedPose p)
        {
            p.parent_frame = parent_frame?.ToRRInfo();
            p.frame = frame?.ToRRInfo();
            p.pose = pose?.ToRRInfo() ?? new com.robotraconteur.geometry.Pose();
        }
        
        public com.robotraconteur.geometry.NamedPose ToRRInfo()
        {
            var p = new com.robotraconteur.geometry.NamedPose();
            CopyTo(p);
            return p;
        }
    }

    public class YamlPose
    {
        public YamlQuaternion orientation { get; set; }
        public YamlPoint position { get; set; }

        public void CopyTo(ref com.robotraconteur.geometry.Pose pose)
        {
            pose.position = position?.ToRRInfo() ?? new com.robotraconteur.geometry.Point() ;
            pose.orientation = orientation?.ToRRInfo() ?? new com.robotraconteur.geometry.Quaternion();
        }

        public com.robotraconteur.geometry.Pose ToRRInfo()
        {
            var p = new com.robotraconteur.geometry.Pose();
            CopyTo(ref p);
            return p;
        }
    }

    public class YamlTransform
    {
        public YamlQuaternion rotation { get; set; }
        public YamlVector3 translation { get; set; }

        public void CopyTo(ref com.robotraconteur.geometry.Transform pose)
        {
            pose.translation = translation?.ToRRInfo() ?? new com.robotraconteur.geometry.Vector3();
            pose.rotation = rotation?.ToRRInfo() ?? new com.robotraconteur.geometry.Quaternion();
        }

        public com.robotraconteur.geometry.Transform ToRRInfo()
        {
            var p = new com.robotraconteur.geometry.Transform();
            CopyTo(ref p);
            return p;
        }
    }

    public class YamlQuaternion
    {
        public double w { get; set; }
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }

        public void CopyTo(com.robotraconteur.geometry.Quaternion q)
        {
            q.w = w;
            q.x = x;
            q.y = y;
            q.z = z;
        }

        public com.robotraconteur.geometry.Quaternion ToRRInfo()
        {
            var q = new com.robotraconteur.geometry.Quaternion();
            CopyTo(q);
            return q;
        }
    }

    public class YamlPoint
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }

        public void CopyTo(ref com.robotraconteur.geometry.Point p)
        {
            p.x = x;
            p.y = y;
            p.z = z;
        }

        public com.robotraconteur.geometry.Point ToRRInfo()
        {
            var p = new com.robotraconteur.geometry.Point();
            CopyTo(ref p);
            return p;
        }
    }

    public class YamlVector3
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }

        public void CopyTo(ref com.robotraconteur.geometry.Vector3 info)
        {
            info.x = x;
            info.y = y;
            info.z = z;
        }

        public com.robotraconteur.geometry.Vector3 ToRRInfo()
        {
            var info = new com.robotraconteur.geometry.Vector3();
            CopyTo(ref info);
            return info;
        }
    }

    public class YamlSpatialInertia
    {
        public double m { get; set; }
        public YamlVector3 com { get; set; }
        public double ixx { get; set; }
        public double ixy { get; set; }
        public double ixz { get; set; }
        public double iyy { get; set; }
        public double iyz { get; set; }
        public double izz { get; set; }

        public void CopyTo(ref com.robotraconteur.geometry.SpatialInertia info)
        {
            info.m = m;
            if (com != null)
            {
                info.com.x = com.x;
                info.com.y = com.y;
                info.com.z = com.z;
            }
            else
            {
                info.com.x = 0;
                info.com.y = 0;
                info.com.z = 0;
            }
            info.ixx = ixx;
            info.ixy = ixy;
            info.ixz = ixz;
            info.iyy = iyy;
            info.iyz = iyz;
            info.izz = izz;
        }

        public com.robotraconteur.geometry.SpatialInertia ToRRInfo()
        {
            var info = new com.robotraconteur.geometry.SpatialInertia();
            CopyTo(ref info);
            return info;
        }
    }

    public class YamlSpatialVelocity
    {
        public YamlVector3 angular { get; set; }
        public YamlVector3 linear { get; set; }

        public void CopyTo(com.robotraconteur.geometry.SpatialVelocity info)
        {
            info.angular = angular?.ToRRInfo() ?? default;
            info.linear = linear?.ToRRInfo() ?? default;
        }

        public com.robotraconteur.geometry.SpatialVelocity ToRRInfo()
        {
            var info = new com.robotraconteur.geometry.SpatialVelocity();
            CopyTo(info);
            return info;
        }
    }

    public class YamlSpatialAcceleration
    {
        public YamlVector3 angular { get; set; }
        public YamlVector3 linear { get; set; }

        public void CopyTo(com.robotraconteur.geometry.SpatialAcceleration info)
        {
            info.angular = angular?.ToRRInfo() ?? default;
            info.linear = linear?.ToRRInfo() ?? default;
        }

        public com.robotraconteur.geometry.SpatialAcceleration ToRRInfo()
        {
            var info = new com.robotraconteur.geometry.SpatialAcceleration();
            CopyTo(info);
            return info;
        }
    }

}
