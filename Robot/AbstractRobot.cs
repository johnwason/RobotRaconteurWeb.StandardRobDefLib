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
using RobotRaconteurWeb;
using com.robotraconteur.robotics.robot;
using System.IO;
using System.Linq;
using com.robotraconteur.robotics.joints;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using com.robotraconteur.geometry;
using com.robotraconteur.action;
using com.robotraconteur.robotics.trajectory;

namespace RobotRaconteurWeb.Robot
{
    public abstract class AbstractRobot : Robot_default_impl, IDisposable
    {        
        protected internal int _joint_count;
        protected internal string[] _joint_names;
        protected const double _jog_joint_limit = Math.PI * (15.0 / 180.0);
        protected const long _jog_joint_timeout = 5000; // milliseconds
        protected const double _jog_joint_tol = Math.PI * (0.1 / 180.0);
        protected internal double _trajectory_error_tol = Math.PI * (5.0 / 180.0);
                
        protected internal RobotCommandMode _command_mode = RobotCommandMode.halt;
        protected RobotOperationalMode _operational_mode = RobotOperationalMode.manual_reduced_speed;
        protected RobotControllerState _controller_state = RobotControllerState.undefined;

        protected double[] _joint_position = new double[0];
        protected double[] _joint_velocity = new double[0];
        protected double[] _joint_effort = new double[0];

        protected double[] _position_command = new double[0];
        protected double[] _velocity_command = new double[0];

        protected Pose? _endpoint_pose;
        protected SpatialVelocity? _endpoint_vel;

        protected internal bool _homed = false;
        protected internal bool _ready = false;
        protected internal bool _enabled = false;
        protected internal bool _stopped = false;
        protected internal bool _error = false;
        protected byte _estop_source = 0;

        protected bool _communication_failure = true;
        protected long _communication_timeout = 250; // milliseconds


        protected Stopwatch _stopwatch;

        protected com.robotraconteur.uuid.UUID _robot_uuid;
        protected com.robotraconteur.robotics.robot.RobotInfo _robot_info;

        protected double _speed_ratio = 1;
        
        public AbstractRobot(com.robotraconteur.robotics.robot.RobotInfo robot_info, int default_joint_count)
        {            
            this._robot_info = robot_info;
            if (robot_info.joint_info != null)
            {
                var j_names = new List<string>();
                foreach (var j_info in robot_info.joint_info)
                {
                    j_names.Add(j_info.joint_identifier.name);
                }
                _joint_names = j_names.ToArray();
            }
            else
            {
                _joint_names = Enumerable.Range(0, default_joint_count).Select(x => $"joint_{x}").ToArray();
            }

            _joint_count = _joint_names.Length;

            _robot_uuid = robot_info.device_info.device.uuid;
        }

        protected bool _keep_going = false;

        public virtual void _start_robot()
        {
            if (!Stopwatch.IsHighResolution)
            {
                Debug.WriteLine("warning: not using high resolution timer");
            }
            _stopwatch = Stopwatch.StartNew();            
                        
            _keep_going = true;
            _loop_thread = new Thread(_loop_thread_func);
            _loop_thread.Start();
        }

        protected virtual void _stop_robot()
        {
            _keep_going = false;
            _loop_thread.Join();
        }

        protected Thread _loop_thread;
        protected virtual void _loop_thread_func()
        { 
            // Use a spin wait loop to get higher timing accurancy
            SpinWait spin_wait = new SpinWait();

            long next_wait = _stopwatch.ElapsedMilliseconds;

            long now = next_wait;

            while (_keep_going)
            {
                _run_timestep(now);

                now = _stopwatch.ElapsedMilliseconds;
                
                do
                {
                    next_wait += 10;
                }
                while (next_wait <= now);

                spin_wait.Reset();
                while ((now = _stopwatch.ElapsedMilliseconds) < next_wait)
                {
                    spin_wait.SpinOnce();
                }
            }
        }

        protected long _last_robot_state;
        protected long _last_joint_state;
        protected long _last_endpoint_state;

        public long _now => _stopwatch.ElapsedMilliseconds;

        
        public virtual void Dispose()
        {
            _keep_going = false;
            
        }

        
        public override Pipe<RobotStateSensorData> robot_state_sensor_data
        {
            get => base.robot_state_sensor_data;
            set
            {
                base.robot_state_sensor_data = value;
                rrvar_robot_state_sensor_data.MaximumBacklog = 3;
            }
        }

        protected internal ulong _state_seqno = 0;

        protected internal virtual void _run_timestep(long now)
        {
            bool res;

            double[] joint_pos_cmd = null;
            double[] joint_vel_cmd = null;

            RobotState rr_robot_state;
            AdvancedRobotState rr_advanced_robot_state;
            RobotStateSensorData rr_state_sensor_data;

            lock (this)
            {
                _state_seqno++;
                
                res = _verify_communication(now);
                res = res && _verify_robot_state(now);
                res = res && _fill_robot_command(now, out joint_pos_cmd, out joint_vel_cmd);

                _fill_states(now, out rr_robot_state, out rr_advanced_robot_state, out rr_state_sensor_data);
            }


            if (!res)
            {
                //_send_disable();
            }
            else
            {
                _send_robot_command(now, joint_pos_cmd, joint_vel_cmd);
            }

            _send_states(now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data);
        }

        protected internal virtual ulong _fill_state_flags(long now)
        {
            ulong f = 0;

            if (_communication_failure)
            {
                f |= (ulong)RobotStateFlags.communication_failure;
                return f;
            }

            if (_error)
            {
                f |= (ulong)RobotStateFlags.error;
            }

            if (_stopped)
            {
                f |= (ulong)RobotStateFlags.estop;

                switch(_estop_source)
                {
                    case 0:
                        break;
                    case 1:
                        f |= (ulong)RobotStateFlags.estop_button1;
                        break;
                    case 2:
                        f |= (ulong)RobotStateFlags.estop_other;
                        break;
                    case 3:
                        f |= (ulong)RobotStateFlags.estop_fault;
                        break;
                    case 4:
                        f |= (ulong)RobotStateFlags.estop_internal;
                        break;
                    default:
                        break;
                }
            }

            if (_enabled)
            {
                f |= (ulong)RobotStateFlags.enabled;
            }

            if (_ready)
            {
                f |= (ulong)RobotStateFlags.ready;
            }

            if (_homed)
            {
                f |= (ulong)RobotStateFlags.homed;
            }
            else
            {
                f |= (ulong)RobotStateFlags.homing_required;
            }

            if (_wire_position_command_sent)
            {
                f |= (ulong)RobotStateFlags.valid_position_command;
            }

            if (_wire_velocity_command_sent)
            {
                f |= (ulong)RobotStateFlags.valid_velocity_command;
            }

            if (_trajectory_valid)
            {
                f |= (ulong)RobotStateFlags.trajectory_running;
            }

            return f;

        }

        protected internal virtual void _fill_states(long now, out RobotState rr_robot_state, out AdvancedRobotState rr_advanced_robot_state, out RobotStateSensorData rr_state_sensor_data)
        {
            var rob_state = new RobotState();
            rob_state.seqno = _state_seqno;
            rob_state.command_mode = _command_mode;
            rob_state.operational_mode = _operational_mode;
            rob_state.controller_state = _controller_state;

            var flags = _fill_state_flags(now);

            rob_state.robot_state_flags = flags;

            rob_state.joint_position = (double[])_joint_position.Clone();
            rob_state.joint_velocity = (double[])_joint_velocity.Clone();
            rob_state.joint_effort = (double[])_joint_effort.Clone();
            rob_state.joint_position_command = new double[0];
            rob_state.joint_velocity_command = new double[0];
            rob_state.kin_chain_tcp = (_endpoint_pose != null) ? new Pose[] { _endpoint_pose.Value } : new Pose[0];
            rob_state.kin_chain_tcp_vel = (_endpoint_vel != null) ? new SpatialVelocity[] { _endpoint_vel.Value } : new SpatialVelocity[0];
            rob_state.trajectory_running = _trajectory_valid;

            var a_rob_state = new AdvancedRobotState();
            a_rob_state.seqno = rob_state.seqno;
            a_rob_state.command_mode = rob_state.command_mode;
            a_rob_state.operational_mode = rob_state.operational_mode;
            a_rob_state.controller_state = rob_state.controller_state;
            a_rob_state.joint_position = rob_state.joint_position;
            a_rob_state.joint_velocity = rob_state.joint_velocity;
            a_rob_state.joint_effort = rob_state.joint_effort;
            a_rob_state.joint_position_command = rob_state.joint_position_command;
            a_rob_state.joint_velocity_command = rob_state.joint_velocity_command;
            a_rob_state.kin_chain_tcp = rob_state.kin_chain_tcp;
            a_rob_state.trajectory_running = rob_state.trajectory_running;
            a_rob_state.joint_position_units = Enumerable.Repeat<byte>((byte)JointPositionUnits.radian, 7).ToArray();
            a_rob_state.joint_effort_units = Enumerable.Repeat<byte>((byte)JointEffortUnits.newton_meter, 7).ToArray();
            a_rob_state.trajectory_running = _trajectory_valid;
            a_rob_state.trajectory_time = _trajectory_current_time;
            a_rob_state.trajectory_max_time = _trajectory_max_time;
            a_rob_state.trajectory_current_waypoint = _trajectory_waypoint;

            var now_utc = new com.robotraconteur.datetime.DateTimeUTC();
            now_utc.clock_info.clock_type = (int)com.robotraconteur.datetime.ClockTypeCode.default_;
            now_utc.clock_info.clock_uuid = _robot_uuid;
            var ts = TimeSpec.Now;
            now_utc.seconds = ts.seconds;
            now_utc.nanoseconds = ts.nanoseconds;

            var sensor_data_header = new com.robotraconteur.sensordata.SensorDataHeader();
            sensor_data_header.seqno = _state_seqno;
            sensor_data_header.ts = now_utc;

            var sensor_data = new RobotStateSensorData();
            sensor_data.data_header = sensor_data_header;
            sensor_data.robot_state = a_rob_state;

            rr_robot_state = rob_state;
            rr_advanced_robot_state = a_rob_state;
            rr_state_sensor_data = sensor_data;
        }

        protected void _send_states(long now, RobotState rr_robot_state, AdvancedRobotState rr_advanced_robot_state, RobotStateSensorData rr_state_sensor_data)
        {
            if (rrvar_robot_state != null)
            {
                rrvar_robot_state.OutValue = rr_robot_state;
            }

            if (rrvar_advanced_robot_state != null)
            {
                rrvar_advanced_robot_state.OutValue = rr_advanced_robot_state;
            }

            rrvar_robot_state_sensor_data?.AsyncSendPacket(rr_state_sensor_data).ContinueWith(t => { var ignore = t.Exception; }, System.Threading.Tasks.TaskContinuationOptions.OnlyOnFaulted);                        
        }

        protected internal abstract Task _send_disable();
        
        public override Task disable(CancellationToken rr_cancel = default)
        {
            return _send_disable();      
        }

        public override Task enable(CancellationToken rr_cancel = default)
        {
            return _send_enable();
        }

        protected internal abstract Task _send_enable();

        public override Task reset_errors(CancellationToken rr_cancel = default)
        {
            return _send_reset_errors();
        }

        protected internal abstract Task _send_reset_errors();


        public override Task halt(CancellationToken rr_cancel = default)
        {
            if (_command_mode == RobotCommandMode.invalid_state)
            {
                return Task.FromResult(0);
            }
            _command_mode = RobotCommandMode.halt;

            return Task.FromResult(0);
        }
        
        protected virtual bool _verify_communication(long now)
        {   
            if (now - _last_joint_state > _communication_timeout
                || now - _last_robot_state > _communication_timeout
                || now - _last_endpoint_state > _communication_timeout)                
            {
                _communication_failure = true;

                _command_mode = RobotCommandMode.invalid_state;
                _operational_mode = RobotOperationalMode.undefined;
                _controller_state = RobotControllerState.undefined;

                _joint_position = new double[0];
                _joint_velocity = new double[0];
                _joint_effort = new double[0];

                _endpoint_pose = null;
                _endpoint_vel = null;

                //_send_disable();

                return false;
            }

            //_operational_mode = RobotOperationalMode.cobot;
                        
            _communication_failure = false;
            return true;
        }

        protected virtual bool _verify_robot_state(long now)
        {
            if (_command_mode == RobotCommandMode.homing)
            {
                if (_enabled && !_error && ! _communication_failure)
                {
                    _controller_state = RobotControllerState.motor_off;
                    return true;
                }
            }

            if (!_ready || _communication_failure)
            {
                if (_stopped)
                {
                    _controller_state = RobotControllerState.emergency_stop;
                }
                else if (_error)
                {
                    _controller_state = RobotControllerState.guard_stop;
                }
                else
                {
                    _controller_state = RobotControllerState.motor_off;
                }

                _command_mode = RobotCommandMode.invalid_state;                
                return false;
            }
            
            if (!_enabled)
            {
                _controller_state = RobotControllerState.motor_off;
                _command_mode = RobotCommandMode.invalid_state;
                return false;
            }


            if (_command_mode == RobotCommandMode.invalid_state)
            {
                _command_mode = RobotCommandMode.halt;
            }

            _controller_state = RobotControllerState.motor_on;

            return true;
        }

        protected bool _wire_position_command_sent;
        protected bool _wire_velocity_command_sent;

        protected ulong _wire_position_command_last_seqno = 0;
        protected ulong _wire_velocity_command_last_seqno = 0;

        protected uint _wire_position_command_last_ep = 0;
        protected uint _wire_velocity_command_last_ep = 0;

        protected bool _trajectory_valid = false;
        protected double _trajectory_current_time;
        protected double _trajectory_max_time;
        protected uint _trajectory_waypoint;

        protected internal virtual bool _fill_robot_command(long now, out double[] joint_pos_cmd, out double[] joint_vel_cmd)
        {
            joint_pos_cmd = null;
            joint_vel_cmd = null;

            _wire_position_command_sent = false;
            _wire_velocity_command_sent = false;

            _trajectory_valid = false;
            _trajectory_current_time = 0;
            _trajectory_max_time = 0;
            _trajectory_waypoint = 0;

            if (_command_mode != RobotCommandMode.trajectory)
            {
                if (_active_trajectory != null)
                {
                    _active_trajectory.invalid_mode();
                    _active_trajectory = null;
                }
                
                if (_queued_trajectories.Count > 0)
                {
                    foreach (var t in _queued_trajectories)
                    {
                        t.invalid_mode();
                    }

                    _queued_trajectories.Clear();
                }
            }

            switch (_command_mode)
            {
                case RobotCommandMode.jog:
                    {
                        if (_jog_command_pos == null || now - _last_jog_command_pos > _jog_joint_timeout)
                        {
                            if (_jog_completion_source != null)
                            {
                                _jog_completion_source.TrySetException(new OperationFailedException("Operation timed out"));
                                _jog_completion_source = null;
                            }
                            
                            return true;
                        }

                        bool within_tol = true;
                        for (int i = 0; i < _joint_count; i++)
                        {
                            if (Math.Abs(_jog_command_pos[i] - _joint_position[i]) > _jog_joint_tol)
                            {
                                within_tol = false;
                            }
                        }

                        if (within_tol)
                        {
                            if (_jog_completion_source != null)
                            {
                                _jog_completion_source.SetResult(0);
                                _jog_completion_source = null;
                            }

                            return true;
                        }

                        
                        joint_pos_cmd = _jog_command_pos;
                                                
                        return true;
                    }
                case RobotCommandMode.position_command:
                    {
                        RobotJointCommand pos_cmd = default;
                        TimeSpec ts;
                        uint ep = default;
                        if (!(rrvar_position_command?.TryGetInValue(out pos_cmd, out ts, out ep) ?? false))
                        {                            
                            return true;
                        }

                        if (_wire_position_command_last_ep != ep)
                        {
                            _wire_position_command_last_ep = ep;
                            _wire_position_command_last_seqno = 0;
                        }

                        if (pos_cmd == null 
                            || pos_cmd.seqno < _wire_position_command_last_seqno
                            || Math.Abs((long)pos_cmd.state_seqno - (long)_state_seqno) > 10
                            || pos_cmd.command.Length != _joint_count
                            || pos_cmd.units.Length != 0 && pos_cmd.units.Length != _joint_count)
                        {
                            
                            return true;
                        }

                        double[] pos_cmd_j;
                        if (pos_cmd.units.Length == 0)
                        {
                            pos_cmd_j = pos_cmd.command;
                        }
                        else
                        {
                            pos_cmd_j = new double[_joint_count];
                            for (int i = 0; i < _joint_count; i++)
                            {
                                switch ((JointPositionUnits)pos_cmd.units[i])
                                {
                                    case JointPositionUnits.implicit_:
                                    case JointPositionUnits.radian:
                                        pos_cmd_j[i] = pos_cmd.command[i];
                                        break;
                                    case JointPositionUnits.degree:
                                        pos_cmd_j[i] = pos_cmd.command[i] * (Math.PI / 180.0);
                                        break;
                                    case JointPositionUnits.ticks_rot:
                                        pos_cmd_j[i] = (pos_cmd.command[i] / (double)(2 ^ 20)) * (Math.PI * 2.0);
                                        break;
                                    case JointPositionUnits.nanoticks_rot:
                                        pos_cmd_j[i] = (pos_cmd.command[i] / (double)(2 ^ 20) * 1e9) * (Math.PI * 2.0);
                                        break;
                                    default:
                                        {
                                            // Invalid units!
                                            
                                            return true;
                                        }
                                }
                            }
                        }                        

                        _wire_position_command_last_seqno = pos_cmd.seqno;
                        
                        joint_pos_cmd = pos_cmd_j;
                        
                        _wire_position_command_sent = true;

                        return true;
                    }
                case RobotCommandMode.velocity_command:
                    {
                        RobotJointCommand vel_cmd = default;
                        TimeSpec ts;
                        uint ep = default;
                        if (!(rrvar_velocity_command?.TryGetInValue(out vel_cmd, out ts, out ep) ?? false))
                        {                            
                            return true;
                        }

                        if (_wire_velocity_command_last_ep != ep)
                        {
                            _wire_velocity_command_last_ep = ep;
                            _wire_velocity_command_last_seqno = 0;
                        }

                        if (vel_cmd == null
                            || vel_cmd.seqno < _wire_velocity_command_last_seqno
                            || Math.Abs((long)vel_cmd.state_seqno - (long)_state_seqno) > 10
                            || vel_cmd.command.Length != _joint_count
                            || vel_cmd.units.Length != 0 && vel_cmd.units.Length != _joint_count)
                        {                            
                            return true;
                        }

                        double[] vel_cmd_j;
                        if (vel_cmd.units.Length == 0)
                        {
                            vel_cmd_j = vel_cmd.command;
                        }
                        else
                        {
                            vel_cmd_j = new double[_joint_count];
                            for (int i = 0; i < _joint_count; i++)
                            {
                                switch ((JointVelocityUnits)vel_cmd.units[i])
                                {
                                    case JointVelocityUnits.implicit_:
                                    case JointVelocityUnits.radian_second:
                                        vel_cmd_j[i] = vel_cmd.command[i];
                                        break;
                                    case JointVelocityUnits.degree_second:
                                        vel_cmd_j[i] = vel_cmd.command[i] * (Math.PI / 180.0);
                                        break;
                                    case JointVelocityUnits.ticks_rot_second:
                                        vel_cmd_j[i] = (vel_cmd.command[i] / (double)(2 ^ 20)) * (Math.PI * 2.0);
                                        break;
                                    case JointVelocityUnits.nanoticks_rot_second:
                                        vel_cmd_j[i] = (vel_cmd.command[i] / (double)(2 ^ 20) * 1e9) * (Math.PI * 2.0);
                                        break;
                                    default:
                                        {
                                            // Invalid units!
                                            return true;
                                        }
                                }
                            }
                        }

                        _wire_velocity_command_last_seqno = vel_cmd.seqno;

                        if (_speed_ratio != 1.0)
                        {
                            for (int i=0; i<vel_cmd_j.Length; i++)
                            {
                                vel_cmd_j[i] = vel_cmd_j[i] * _speed_ratio;
                            }
                        }
                                                
                        joint_vel_cmd = vel_cmd_j;

                        _wire_velocity_command_sent = true;

                        return true;
                    }
                case RobotCommandMode.trajectory:
                    {
                        if (_active_trajectory != null)
                        {
                            bool send_traj_cmd;
                            var interp_res = _active_trajectory.get_setpoint(now, _joint_position, out var traj_pos, out var traj_vel, out var traj_t, out var traj_max_time, out var traj_waypoint);
                            switch (interp_res)
                            {
                                case TrajectoryTaskRes.ready:
                                    _trajectory_valid = true;
                                    send_traj_cmd = false;
                                    break;
                                case TrajectoryTaskRes.first_valid_setpoint:
                                case TrajectoryTaskRes.valid_setpoint:
                                    _trajectory_valid = true;
                                    send_traj_cmd = true;
                                    break;
                                case TrajectoryTaskRes.trajectory_complete:
                                    _trajectory_valid = true;
                                    send_traj_cmd = true;
                                    _active_trajectory = null;
                                    if (_queued_trajectories.Count >0)
                                    {
                                        _active_trajectory = _queued_trajectories[0];
                                        _queued_trajectories.RemoveAt(0);
                                    }
                                    break;
                                default:
                                    _trajectory_valid = false;
                                    send_traj_cmd = false;
                                    _active_trajectory = null;
                                    foreach (var w in _queued_trajectories)
                                    {
                                        w._cancelled_in_queue();
                                    }
                                    _queued_trajectories.Clear();
                                    break;
                            }

                            if (_trajectory_valid)
                            {
                                _trajectory_current_time = traj_t;
                                _trajectory_max_time = traj_max_time;
                                _trajectory_waypoint = (uint)traj_waypoint;
                            }

                            if (send_traj_cmd)
                            {
                                joint_pos_cmd = traj_pos;
                            }
                            else
                            {
                                joint_pos_cmd = null;
                            }
                        }
                        else
                        {
                            joint_pos_cmd = null;
                        }
                        return true;
                    }
                default:
                    {
                        joint_pos_cmd = null;
                        return true;
                    }
            }
        }

        protected abstract void _send_robot_command(long now, double[] joint_pos_cmd, double[] joint_vel_cmd);
       

        public override Task<RobotCommandMode> get_command_mode(CancellationToken cancel = default)
        {
            lock (this)
            {
                return Task.FromResult(_command_mode);
            }
        }
                
        public override Task set_command_mode(RobotCommandMode value, CancellationToken cancel = default)
        {
            lock (this)
            {
                if (_command_mode == RobotCommandMode.invalid_state && value == RobotCommandMode.homing)
                {
                    if (!_enabled || _communication_failure)
                    {
                        throw new InvalidOperationException("Cannot set homing command mode in current state");
                    }

                    _command_mode = RobotCommandMode.homing;
                    return Task.FromResult(0);
                }

                if (!_ready || _communication_failure)
                {
                    throw new InvalidOperationException("Cannot set robot command mode in current state");
                }

                if (_command_mode != RobotCommandMode.halt && value != RobotCommandMode.halt)
                {
                    throw new InvalidOperationException("Must switch to \"halt\" before selecting new mode");
                }

                switch (value)
                {
                    case RobotCommandMode.jog:
                        {
                            _jog_command_pos = null;
                            _command_mode = RobotCommandMode.jog;
                            break;
                        }
                    case RobotCommandMode.halt:
                    case RobotCommandMode.homing:
                    case RobotCommandMode.position_command:
                    case RobotCommandMode.velocity_command:
                    case RobotCommandMode.trajectory:
                        _command_mode = value;
                        break;
                    default:
                        throw new ArgumentException("Invalid command mode specified");
                }
            }

            return Task.FromResult(0);
        }

        protected double[] _jog_command_pos = null;
        protected long _last_jog_command_pos = 0;
        protected TaskCompletionSource<int> _jog_completion_source;


        public override Task jog_joint(double[] joint_position, double[] max_velocity, bool relative, bool wait, CancellationToken rr_cancel = default)
        {
            lock(this)
            {
                if (_command_mode != RobotCommandMode.jog)
                {
                    throw new InvalidOperationException("Robot not in jog mode");
                }

                if (!_ready || _joint_position.Length != _joint_count)
                {
                    throw new OperationAbortedException("Robot not ready");
                }
                
                if (joint_position.Length != _joint_count)
                {
                    throw new ArgumentException("joint_position array must have 7 elements");
                }

                if (max_velocity.Length != _joint_count)
                {
                    throw new ArgumentException("max_velocity array must have 7 elements");
                }

                double[] joint_position2;
                if (relative)
                {
                    joint_position2 = (double[])_joint_position.Clone();
                    for (int i=0; i<_joint_count; i++)
                    {
                        joint_position2[i] += joint_position[i];
                    }
                }
                else
                {
                    joint_position2 = joint_position;
                }

                for (int i=0; i<_joint_count; i++)
                {
                    if (Math.Abs(_joint_position[i] - joint_position2[i]) > _jog_joint_limit)
                    {
                        throw new ArgumentException("Position command must be within 15 degrees from current");
                    }
                }


                if (_jog_completion_source != null)
                {
                    _jog_completion_source.TrySetException(new OperationAbortedException("Operation interrupted by new jog command"));
                    _jog_completion_source = null;
                }

                long now = _stopwatch.ElapsedMilliseconds;

                _jog_command_pos = joint_position2;
                _last_jog_command_pos = now;

                if (!wait)
                {
                    _jog_completion_source = null;
                    return Task.FromResult(0);
                }
                else
                {
                    _jog_completion_source = new TaskCompletionSource<int>();
                    return _jog_completion_source.Task;
                }
            }
        }

        public override Task<RobotInfo> get_robot_info(CancellationToken cancel = default)
        {
            return Task.FromResult(_robot_info);
        }

        protected TrajectoryTask _active_trajectory;
        protected List<TrajectoryTask> _queued_trajectories = new List<TrajectoryTask>();
        public override async Task<Generator2<com.robotraconteur.robotics.trajectory.TrajectoryStatus>> execute_trajectory(com.robotraconteur.robotics.trajectory.JointTrajectory trajectory, CancellationToken rr_cancel = default)
        {
            
            uint owner_ep = ServerEndpoint.CurrentEndpoint.LocalEndpoint;


            double[] current_joint_pos;
            double speed_ratio;
            lock(this)
            {
                if (_command_mode != RobotCommandMode.trajectory)
                {
                    throw new InvalidOperationException("Robot must be in trajectory mode to execute trajectory");
                }

                speed_ratio = _speed_ratio;
                current_joint_pos = _joint_position;
            }
            
            var interp = await Task.Run(delegate ()
            {
                var interp1 = new JointTrajectoryInterpolator(_robot_info);
                interp1.LoadTrajectory(trajectory, speed_ratio);
                return interp1;
            });

            interp.Intepolate(0, out var joint_pos1, out var current_waypoint1);

            for (int i = 0; i < current_joint_pos.Length; i++)
            {
                if (Math.Abs(current_joint_pos[i] - joint_pos1[i]) > _trajectory_error_tol)
                {
                    throw new ArgumentException("Starting waypoint too far from current joint positions");
                }
            }

            lock (this)
            {
                if (_command_mode != RobotCommandMode.trajectory)
                {
                    throw new InvalidOperationException("Robot must be in trajectory mode to execute trajectory");
                }

                TrajectoryTask traj_task;

                if (_active_trajectory == null)
                {
                    traj_task = new TrajectoryTask(this, interp, false, owner_ep);
                    _active_trajectory = traj_task;
                }
                else
                {
                    traj_task = new TrajectoryTask(this, interp, true, owner_ep);
                    _queued_trajectories.Add(traj_task);
                }

                return traj_task;
            }

            throw new NotImplementedException("Trajectory passed checks");

        }

        protected internal virtual void _cancel_trajectory(TrajectoryTask trajectory)
        {
            lock (this)
            {
                if (trajectory.Equals(_active_trajectory))
                {
                    _active_trajectory = null;
                    foreach (var t in _queued_trajectories)
                    {
                        t._cancelled_in_queue();
                    }
                    _queued_trajectories.Clear();
                }
                else
                {
                    int t_index = -1;
                    for (int i = 0; i<_queued_trajectories.Count; i++)
                    {
                        if (trajectory.Equals(_queued_trajectories[i]))
                        {
                            t_index = i;
                        }
                    }

                    for (int i = _queued_trajectories.Count - 1; i > t_index; i--)
                    {
                        _queued_trajectories[i]._cancelled_in_queue();
                        _queued_trajectories.RemoveAt(i);
                    }

                    _queued_trajectories.RemoveAt(t_index);
                }
            }
        }

        internal protected void _abort_trajectory(TrajectoryTask trajectory)
        {
            _command_mode = RobotCommandMode.halt;
        }
        
        public override Task<double> get_speed_ratio(CancellationToken cancel = default)
        {
            return Task.FromResult(_speed_ratio);
        }

        public override Task set_speed_ratio(double value, CancellationToken cancel = default)
        {
            if (value < 0.1 || value > 10)
            {
                throw new ArgumentException("Invalid speed_ration");
            }

            _speed_ratio = value;
            return Task.FromResult(0);
        }
    }
    
    enum TrajectoryTaskRes
    {
        unknown = 0,
        ready,
        first_valid_setpoint,
        valid_setpoint,
        trajectory_complete,
        invalid_state,
        joint_tol_error,
        failed
    }

    public class TrajectoryTask : Generator2<com.robotraconteur.robotics.trajectory.TrajectoryStatus>
    {

        AbstractRobot parent;
        JointTrajectoryInterpolator path;
        bool next_called = false;
        bool started = false;
        long start_time = 0;
        bool aborted = false;
        bool cancelled = false;
        bool joint_tol_error = false;
        bool finished = false;
        TaskCompletionSource<int> next_wait = new TaskCompletionSource<int>();
        TaskCompletionSource<int> queue_wait = new TaskCompletionSource<int>();
        bool queued;

        uint owner_ep;

        RobotRaconteurNode node;

        public TrajectoryTask(AbstractRobot parent, JointTrajectoryInterpolator path, bool queued, uint owner_ep)
        {
            this.parent = parent;
            this.path = path;
            this.queued = queued;

            this.owner_ep = owner_ep;
            //TODO: find out which node is being used
            this.node = RobotRaconteurNode.s;

            connection_test().ContinueWith(t => { });
        }


        public Task Abort(CancellationToken cancel = default)
        {
            aborted = true;
            parent._abort_trajectory(this);
            next_wait.TrySetException(new OperationAbortedException("Trajectory execution aborted"));
            return Task.FromResult(0);
        }

        public Task Close(CancellationToken cancel = default)
        {
            cancelled = true;
            parent._cancel_trajectory(this);
            next_wait.TrySetException(new OperationAbortedException("Trajectory execution cancelled"));
            return Task.FromResult(0);
        }

        bool success_sent = false;
        public async Task<TrajectoryStatus> Next(CancellationToken cancel = default)
        {
            if (success_sent)
            {
                throw new StopIterationException("");
            }

            lock(this)
            {
                bool first_call = false;
                if (!next_called)
                {
                    first_call = true;
                }
                next_called = true;

                if (first_call && queued)
                {
                    //Report back that we are queued immediately
                    var ret = new TrajectoryStatus();
                    ret.action_status = ActionStatusCode.queued;
                    ret.trajectory_time = 0;
                    ret.current_waypoint = 0;
                    ret.seqno = parent._state_seqno;
                    return ret;
                }


            }

            Task wait_task;
            if (queued)
            {
                wait_task = await Task.WhenAny(Task.Delay(5000), next_wait.Task, queue_wait.Task);
            }
            else
            {
                wait_task = await Task.WhenAny(Task.Delay(5000), next_wait.Task);
            }
            await wait_task;

            if (!this.started)
            {
                // Still queued...
                var ret = new TrajectoryStatus();
                ret.action_status = ActionStatusCode.queued;
                ret.trajectory_time = 0;
                ret.current_waypoint = 0;
                ret.seqno = parent._state_seqno;
                return ret;
            }

            if (finished)
            {
                success_sent = true;
                var ret = new TrajectoryStatus();
                ret.action_status = ActionStatusCode.complete;
                ret.trajectory_time = traj_t;
                ret.current_waypoint = (uint)traj_waypoint;
                ret.seqno = parent._state_seqno;
                return ret;
            }
            else
            {
                var ret = new TrajectoryStatus();
                ret.action_status = ActionStatusCode.running;
                ret.trajectory_time = traj_t;
                ret.current_waypoint = (uint)traj_waypoint;
                ret.seqno = parent._state_seqno;
                return ret;
            }
        }

        public Task<TrajectoryStatus[]> NextAll(CancellationToken cancel = default)
        {
            throw new NotImplementedException();
        }

        internal void _cancelled_in_queue()
        {
            cancelled = true;
            next_wait.TrySetException(new OperationAbortedException("Trajectory cancelled by controller before start"));
        }

        internal void invalid_mode()
        {
            aborted = true;            
            next_wait.TrySetException(new OperationAbortedException("Invalid mode for trajectory execution"));     
        }

        double traj_t = 0;
        int traj_waypoint = 0;
        internal TrajectoryTaskRes get_setpoint(long now, double[] current_joint_pos, out double[] joint_pos, out double[] joint_vel, out double trajectory_time, out double trajectory_max_time, out int current_waypoint)
        {
            if (cancelled || aborted)
            {
                joint_pos = null;
                joint_vel = null;
                trajectory_time = 0;
                current_waypoint = 0;
                trajectory_max_time = 0;
                return TrajectoryTaskRes.failed;
            }

            bool first_call = false;

            double t = 0;

            if (next_called)
            {
                if (!started)
                {
                    start_time = now;
                    started = true;
                    first_call = true;
                }

                long t_long = now - start_time;
                t = ((double)t_long) / 1000.0;
            }

            this.path.Intepolate(t, out var joint_pos1, out var current_waypoint1);

            for (int i=0; i< current_joint_pos.Length; i++)
            {
                if (Math.Abs(current_joint_pos[i] - joint_pos1[i]) > parent._trajectory_error_tol)
                {
                    joint_tol_error = true;
                    joint_pos = null;
                    joint_vel = null;
                    trajectory_time = 0;
                    current_waypoint = 0;
                    trajectory_max_time = 0;
                    next_wait.TrySetException(new OperationFailedException("Trajectory tolerance failure"));
                    return TrajectoryTaskRes.joint_tol_error;
                }
            }

            if (!next_called)
            {
                joint_pos = null;
                joint_vel = null;
                trajectory_time = 0;
                current_waypoint = 0;
                trajectory_max_time = path.MaxTime;
                return TrajectoryTaskRes.ready;
            }

            joint_pos = joint_pos1;
            joint_vel = null;
            trajectory_time = t;
            traj_t = t;
            traj_waypoint = current_waypoint1;
            current_waypoint = current_waypoint1;
            trajectory_max_time = path.MaxTime;

            if (t > path.MaxTime)
            {
                trajectory_time = path.MaxTime;
                traj_t = path.MaxTime;
                finished = true;                
                next_wait.TrySetResult(0);
                return TrajectoryTaskRes.trajectory_complete;
            }

            if (first_call)
            {
                if (queued)
                {
                    queued = false;
                    queue_wait.TrySetResult(0);
                }
                return TrajectoryTaskRes.first_valid_setpoint;
            }
            else
            {
                return TrajectoryTaskRes.valid_setpoint;
            }
        }

        protected async Task connection_test()
        {
            while (!finished)
            {
                try
                {
                    node.CheckConnection(owner_ep);
                }
                catch (Exception)
                {
                    parent._cancel_trajectory(this);
                    next_wait.TrySetException(new ConnectionException("Connection lost"));
                }
                await Task.Delay(50);
            }
        }
    }
}
