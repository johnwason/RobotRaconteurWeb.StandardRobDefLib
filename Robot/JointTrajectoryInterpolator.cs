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
using System.Linq;
using MathNet.Numerics.Interpolation;

namespace RobotRaconteurWeb.StandardRobDefLib.Robot
{
    public class JointTrajectoryInterpolator
    {
        string[] joint_names;
        double[] joint_min;
        double[] joint_max;
        double[] joint_vel_max;

        List<CubicSpline> joint_splines;
        double max_t;

        double[] joint_start;
        double[] joint_end;
        double[] waypoint_times;

        public JointTrajectoryInterpolator(com.robotraconteur.robotics.robot.RobotInfo info)
        {
            joint_names = info.joint_info.Select(x => x.joint_identifier.name).ToArray();
            joint_min = info.joint_info.Select(x => x.joint_limits.lower).ToArray();
            joint_max = info.joint_info.Select(x => x.joint_limits.upper).ToArray();
            joint_vel_max = info.joint_info.Select(x => x.joint_limits.velocity).ToArray();
        }

        public void LoadTrajectory(com.robotraconteur.robotics.trajectory.JointTrajectory traj, double speed_ratio)
        {
            if (traj.joint_names.Count > 0)
            {
                if (!Enumerable.SequenceEqual(traj.joint_names, joint_names))
                {
                    throw new ArgumentException("Joint names in trajectory must match robot joint names");
                }
            }

            if (traj.waypoints == null)
            {
                throw new ArgumentException("Waypoint list must not be null");
            }

            if (traj.waypoints.Count < 2)
            {
                throw new ArgumentException("Waypoint list must contain two or more waypoints");
            }

            if (traj.waypoints[0].time_from_start != 0)
            {
                throw new ArgumentException("Trajectory time_from_start must equal zero for first waypoint");
            }

            if (traj.interpolation_mode != com.robotraconteur.robotics.trajectory.InterpolationMode.default_
                    && traj.interpolation_mode != com.robotraconteur.robotics.trajectory.InterpolationMode.cubic_spline)
            {
                throw new ArgumentException($"Only default and cubic_spline interpolation supported for trajectory execution");
            }

            int n_waypoints = traj.waypoints.Count;
            int n_joints = joint_names.Length;

            var traj_t = new double[n_waypoints];
            List<double[]> traj_j = Enumerable.Range(1,n_joints).Select(i => new double[n_waypoints]).ToList();

            double last_t = 0;
                        
            for (int i = 0; i < n_waypoints; i++)
            {
                var w = traj.waypoints[i];

                if (w.joint_position.Length != n_joints)
                {
                    throw new ArgumentException($"Waypoint {i} invalid joint array length");
                }

                if (w.joint_velocity.Length != n_joints && w.joint_velocity.Length != 0)
                {
                    throw new ArgumentException($"Waypoint {i} invalid joint velocity array length");
                }

                if (w.position_tolerance.Length != n_joints && w.position_tolerance.Length != 0)
                {
                    throw new ArgumentException($"Waypoint {i} invalid tolerance array length");
                }

                if (w.velocity_tolerance.Length != n_joints && w.velocity_tolerance.Length != 0)
                {
                    throw new ArgumentException($"Waypoint {i} invalid tolerance array length");
                }
                                
                if (i > 0)
                {
                    if (w.time_from_start/speed_ratio <= last_t)
                    {
                        throw new ArgumentException("time_from_start must be increasing");
                    }
                }
                                
                for (int j = 0; j < n_joints; j++)
                {
                    if (w.joint_position[j] > joint_max[j] || w.joint_position[j] < joint_min[j])
                    {
                        throw new ArgumentException($"Waypoint {i} exceeds joint limits");
                    }

                    if (w.joint_velocity.Length > 0)
                    {
                        if (Math.Abs(w.joint_velocity[j]*speed_ratio) > joint_vel_max[j])
                        {
                            throw new ArgumentException($"Waypoint {i} exceeds joint velocity limits");
                        }
                    }
                }

                if (i > 0)
                {
                    var last_w = traj.waypoints[i - 1];
                    double dt = w.time_from_start/speed_ratio - last_w.time_from_start/speed_ratio;

                    for (int j = 0; j < n_joints; j++)
                    {
                        double dj = Math.Abs(w.joint_position[j] - last_w.joint_position[j]);
                        if (dj/dt > joint_vel_max[j])
                        {
                            throw new ArgumentException($"Waypoint {i} exceeds joint velocity limits");
                        }
                    }
                }

                traj_t[i] = w.time_from_start/speed_ratio;
                for (int j = 0; j < n_joints; j++)
                {
                    traj_j[j][i] = w.joint_position[j];
                }

                last_t = w.time_from_start/speed_ratio;
            }

            joint_splines = traj_j.Select(x => CubicSpline.InterpolateAkima(traj_t, x)).ToList();
            max_t = last_t;

            joint_start = traj.waypoints[0].joint_position;
            joint_end = traj.waypoints.Last().joint_position;
            waypoint_times = traj_t;
        }

        public double MaxTime => max_t;

        public bool Intepolate(double time, out double[] joint_pos, out int current_waypoint)
        {
            if (time <= 0)
            {
                joint_pos = joint_start;
                current_waypoint = 0;
                return true;
            }
            
            if (time >= max_t)
            {
                joint_pos = joint_end;
                current_waypoint = waypoint_times.Length - 1;
                return true;
            }

            var ret = new double[joint_splines.Count];
            for (int i = 0; i< joint_splines.Count; i++)
            {
                ret[i] = joint_splines[i].Interpolate(time);
            }

            joint_pos = ret;
            current_waypoint = 0;
            for (int i=0; i<joint_splines.Count; i++)
            {
                if (waypoint_times[i] > time)
                {
                    break;
                }
                else
                {
                    current_waypoint = i;
                }
            }

            return true;
        }

    }
}
