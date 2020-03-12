using System;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using com.robotraconteur.geometry;
using System.Diagnostics;
using Rox = GeneralRoboticsToolbox;
using static GeneralRoboticsToolbox.Functions;

namespace RobotRaconteurWeb.StandardRobDefLib.Converters
{
    public static class GeometryConverter
    {
        public static Vector2 ToVector2(Vector<double> vs)
        {
            Debug.Assert(vs.Count == 2);
            return new Vector2() { x = vs[0], y = vs[1] };            
        }

        public static Vector<double> ToVector(Vector2 vs)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { vs.x, vs.y});
        }

        public static Vector3 ToVector3(Vector<double> vs)
        {
            Debug.Assert(vs.Count == 3);
            return new Vector3() { x = vs[0], y = vs[1], z = vs[2] };
        }

        public static Vector<double> ToVector(Vector3 vs)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { vs.x, vs.y, vs.z });
        }

        public static Vector6 ToVector6(Vector<double> vs)
        {
            Debug.Assert(vs.Count == 6);
            return new Vector6() {alpha = vs[0], beta = vs[1], gamma = vs[2], x = vs[3], y = vs[4], z = vs[5] };
        }

        public static Vector<double> ToVector(Vector6 vs)
        {
            return Vector<double>.Build.DenseOfArray(new double[] {vs.alpha, vs.beta, vs.gamma, vs.x, vs.y, vs.z });
        }

        public static Point2D ToPoint2D(Vector<double> vs)
        {
            Debug.Assert(vs.Count == 2);
            return new Point2D() { x = vs[0], y = vs[1] };
        }

        public static Vector<double> ToVector(Point2D vs)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { vs.x, vs.y });
        }

        public static Point ToPoint(Vector<double> vs)
        {
            Debug.Assert(vs.Count == 3);
            return new Point() { x = vs[0], y = vs[1], z=vs[2] };
        }

        public static Vector<double> ToVector(Point vs)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { vs.x, vs.y, vs.z });
        }

        public static Size2D ToSize2D(Vector<double> vs)
        {
            Debug.Assert(vs.Count == 2);
            return new Size2D() { width = vs[0], height = vs[1] };
        }

        public static Vector<double> ToVector(Size2D vs)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { vs.width, vs.height });
        }

        public static Size ToSize(Vector<double> vs)
        {
            Debug.Assert(vs.Count == 3);
            return new Size() { width = vs[0], height = vs[1], depth = vs[2] };
        }

        public static Vector<double> ToVector(Size vs)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { vs.width, vs.height, vs.depth });
        }

        public static Quaternion ToQuaternion(Vector<double> quat)
        {
            Debug.Assert(quat.Count == 4);
            return new Quaternion() { w = quat[0], x = quat[1], y = quat[2], z = quat[3] };
        }

        public static Vector<double> ToVector(Quaternion quat)
        {
            if (quat.x == 0 && quat.y == 0 && quat.z == 0 && quat.w == 0)
            {
                return Vector<double>.Build.DenseOfArray(new double[] { 1.0, 0.0, 0.0, 0.0 });
            }
            return Vector<double>.Build.DenseOfArray(new double[] { quat.w, quat.x, quat.y, quat.z });
        }

        public static Rox.Transform ToTransform(Transform tf)
        {
            return new Rox.Transform(Q2R(ToVector(tf.rotation)), ToVector(tf.translation));
        }

        public static Transform ToTransform(Rox.Transform tf)
        {
            return new Transform() { rotation = ToQuaternion(R2Q(tf.R)), translation = ToVector3(tf.P) };            
        }

        public static Rox.Transform ToTransform(NamedTransform tf)
        {
            return new Rox.Transform(Q2R(ToVector(tf.transform.rotation)), ToVector(tf.transform.translation), tf.parent_frame.name, tf.child_frame.name);
        }

        public static NamedTransform ToNamedTransform(Rox.Transform tf)
        {
            var o = new NamedTransform();
            o.transform = ToTransform(tf);
            o.child_frame.name = tf.Child_frame_id ?? "";
            o.parent_frame.name = tf.Parent_frame_id ?? "";
            return o;
        }

        public static Rox.Transform ToTransform(Pose tf)
        {
            return new Rox.Transform(Q2R(ToVector(tf.orientation)), ToVector(tf.position));
        }

        public static Pose ToPose(Rox.Transform tf)
        {
            return new Pose() { orientation = ToQuaternion(R2Q(tf.R)), position = ToPoint(tf.P) };
        }

        public static Rox.Transform ToTransform(NamedPose tf)
        {
            return new Rox.Transform(Q2R(ToVector(tf.pose.orientation)), ToVector(tf.pose.position), tf.parent_frame.name, tf.frame.name);
        }

        public static NamedPose ToNamedPose(Rox.Transform tf)
        {
            var o = new NamedPose();
            o.pose = ToPose(tf);
            o.frame.name = tf.Child_frame_id ?? "";
            o.parent_frame.name = tf.Parent_frame_id ?? "";
            return o;
        }

        public static Vector<double> ToVector(SpatialVelocity v)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { v.angular.x, v.angular.y, v.angular.z, v.linear.x, v.linear.y, v.linear.z });
        }

        public static SpatialVelocity ToSpatialVelocity(Vector<double> v)
        {
            Debug.Assert(v.Count == 6);
            return new SpatialVelocity()
            {
                angular = new Vector3() { x = v[0], y = v[1], z = v[2] },
                linear = new Vector3() { x = v[3], y = v[4], z = v[5] }
            };
        }

        public static Vector<double> ToVector(SpatialAcceleration v)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { v.angular.x, v.angular.y, v.angular.z, v.linear.x, v.linear.y, v.linear.z });
        }

        public static SpatialAcceleration ToSpatialAcceleration(Vector<double> v)
        {
            Debug.Assert(v.Count == 6);
            return new SpatialAcceleration()
            {
                angular = new Vector3() { x = v[0], y = v[1], z = v[2] },
                linear = new Vector3() { x = v[3], y = v[4], z = v[5] }
            };
        }

        public static Vector<double> ToVector(Wrench v)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { v.torque.x, v.torque.y, v.torque.z, v.force.x, v.force.y, v.force.z });
        }

        public static Wrench ToWrench(Vector<double> v)
        {
            Debug.Assert(v.Count == 6);
            return new Wrench()
            {
                torque = new Vector3() { x = v[0], y = v[1], z = v[2] },
                force = new Vector3() { x = v[3], y = v[4], z = v[5] }
            };
        }

        // TODO: SpatialInertia


    }
}
