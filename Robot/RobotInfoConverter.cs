using System;
using System.Collections.Generic;
using System.Text;
using com.robotraconteur.robotics.robot;
using Rox = GeneralRoboticsToolbox;
using static GeneralRoboticsToolbox.Functions;
using MathNet.Numerics.LinearAlgebra;
using System.Linq;
using RobotRaconteurWeb.StandardRobDefLib.Converters;

namespace RobotRaconteurWeb.StandardRobDefLib.Robot
{
    public static class RobotInfoConverter
    {

        private static void CheckList<T>(IList<T> l, string error_msg, int expected_count = -1)
        {
            if (l == null)
            {
                throw new ArgumentException(error_msg);
            }

            if (expected_count >= 0)
            {
                if (l.Count != expected_count)
                {
                    throw new ArgumentException(error_msg);
                }
            }
        }

        public static Rox.Robot ToToolboxRobot(RobotInfo robot_info, int chain_number)
        {
            CheckList(robot_info.chains, $"could not find kinematic chain number {chain_number}");
            if (chain_number >= robot_info.chains.Count) 
                throw new ArgumentException($"invalid kinematic chain number {chain_number}");

            var chain = robot_info.chains[chain_number];
            int joint_count = chain.joint_numbers.Length;
            for (int i=1; i<joint_count; i++)
            {
                if (chain.joint_numbers[i-1] >= chain.joint_numbers[i])
                {
                    throw new ArgumentException($"joint numbers must be increasing in chain number {chain_number}");
                }

                if (chain.joint_numbers[i] >= robot_info.joint_info.Count)
                {
                    throw new ArgumentException($"joint number out of bounds in chain number {chain_number}");
                }
            }

            CheckList(chain.H, $"invalid shape for H in chain number {chain_number}", joint_count);
            CheckList(chain.P, $"invalid shape for P in chain number {chain_number}", joint_count + 1);

            var H = Matrix<double>.Build.Dense(3, joint_count);
            for (int i=0; i<joint_count; i++)
            {
                H[0, i] = chain.H[i].x;
                H[1, i] = chain.H[i].y;
                H[2, i] = chain.H[i].z;
            }

            var P = Matrix<double>.Build.Dense(3, joint_count + 1);
            for (int i = 0; i < joint_count + 1; i++)
            {
                P[0, i] = chain.P[i].x;
                P[1, i] = chain.P[i].y;
                P[2, i] = chain.P[i].z;
            }

            var joint_type = new Rox.JointType[joint_count];
            var joint_lower_limit = new double[joint_count];
            var joint_upper_limit = new double[joint_count];
            var joint_vel_limit = new double[joint_count];
            var joint_acc_limit = new double[joint_count];
            var joint_names = new string[joint_count];

            for (int i=0; i<joint_count; i++)
            {
                var j = robot_info.joint_info[i];
                switch(j.joint_type)
                {
                    case com.robotraconteur.robotics.joints.JointType.revolute:
                        joint_type[i] = Rox.JointType.Revolute;
                        break;
                    case com.robotraconteur.robotics.joints.JointType.prismatic:
                        joint_type[i] = Rox.JointType.Prismatic;
                        break;
                    default:
                        throw new ArgumentException($"invalid joint type: {j.joint_type}");                        
                }

                if (j.joint_limits == null)
                {
                    throw new ArgumentException("joint_limits must not be null");
                }
                joint_lower_limit[i] = j.joint_limits.lower;
                joint_upper_limit[i] = j.joint_limits.upper;
                joint_vel_limit[i] = j.joint_limits.velocity;
                joint_acc_limit[i] = j.joint_limits.acceleration;
                joint_names[i] = j.joint_identifier?.name ?? "";
            }

            string root_link_name = chain.link_identifiers?.FirstOrDefault()?.name;
            string tip_link_name = chain.flange_identifier?.name;

            var tf_tool = GeometryConverter.ToTransform(chain.flange_pose);

            var r_tool = tf_tool.R;
            var p_tool = tf_tool.P;

            var rox_robot = new Rox.Robot(H, P, joint_type, joint_lower_limit, joint_upper_limit, joint_vel_limit,
                joint_acc_limit, null, r_tool, p_tool, joint_names, root_link_name, tip_link_name);

            return rox_robot;


        }

    }
}
