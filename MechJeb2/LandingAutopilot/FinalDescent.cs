using System;
using System.Collections.Generic;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class FinalDescent : AutopilotStep
        {
            PIDController vert_speed_ctrl = new PIDController(2, 0.5, 0);
            PIDControllerV lateral_speed_ctrl = new PIDControllerV(0.1, 0, 0.4);

            List<KeyValuePair<float, AbsoluteVector>> vertical_velocity_profile;
            readonly ReferenceFrame frame;

            public FinalDescent(MechJebCore core, List<TrajectoryPoint> trajectory, ReferenceFrame frame) : base(core)
            {
                // FIXME: use the simulator result
                vertical_velocity_profile = new List<KeyValuePair<float, AbsoluteVector>>();
                this.frame = frame;

                /*double vertical_accel = vesselState.limitedMaxThrustAccel * 0.9 - mainBody.GeeASL * 9.81;
                if (vertical_accel > 0)
                {
                    for (float h = (float)vesselState.altitudeTrue; h > 0; h = h - 1)
                    {
                        double v = -Math.Sqrt(vertical_accel * 2 * h + Math.Pow(core.landing.touchdownSpeed, 2));
                        vertical_velocity_profile.Add(new KeyValuePair<float, float>(h, (float)v));
                    }
                    // new GravityTurnDescentSpeedPolicy(terrainRadius, mainBody.GeeASL * 9.81, vesselState.limitedMaxThrustAccel);
                }
                else
                {
                    vertical_velocity_profile.Add(new KeyValuePair<float, float>(1, 0));
                    vertical_velocity_profile.Add(new KeyValuePair<float, float>(0, 0));
                }*/

                foreach (var i in trajectory)
                {
                    vertical_velocity_profile.Add(new KeyValuePair<float, AbsoluteVector>((float)(i.pos.radius - mainBody.Radius), i.svel));
                }

                core.warp.MinimumWarp(true);
            }

            // Finds the target vertical velocity for a given time relative to the current time
            Vector3d TargetVelocity(float dt)
            {
                Vector3d pos = vesselState.CoM - vessel.mainBody.position + dt * vesselState.surfaceVelocity;
                float alt = (float)(pos.magnitude - mainBody.Radius);

                if (alt > vertical_velocity_profile[0].Key)
                    return frame.WorldVelocityAtCurrentTime(vertical_velocity_profile[0].Value);

                if (alt < vertical_velocity_profile[vertical_velocity_profile.Count - 1].Key)
                    return frame.WorldVelocityAtCurrentTime(vertical_velocity_profile[vertical_velocity_profile.Count - 1].Value);

                int i1 = 0;
                int i2 = vertical_velocity_profile.Count - 1;
                while (i2 - i1 > 1)
                {
                    int i = (i1 + i2) / 2;
                    if (vertical_velocity_profile[i].Key < alt)
                        i2 = i;
                    else
                        i1 = i;
                }

                Vector3d v1 = frame.WorldVelocityAtCurrentTime(vertical_velocity_profile[i1].Value);
                Vector3d v2 = frame.WorldVelocityAtCurrentTime(vertical_velocity_profile[i2].Value);

                float dh = vertical_velocity_profile[i2].Key - vertical_velocity_profile[i1].Key;
                Vector3d dv = v2 - v1;
                return v1 + (alt - vertical_velocity_profile[i1].Key) * dv / dh;
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (vessel.LandedOrSplashed)
                {
                    core.landing.StopLanding();
                    return null;
                }

                // TODO perhaps we should pop the parachutes at this point, or at least consider it depending on the altitude.

                double minalt = core.landing.MinAltitude();
                status = "Final descent: " + minalt.ToString("F0") + "m above terrain";

                float dt = 0.1f;
                Vector3d target_vel = TargetVelocity(0);
                Vector3d target_vel2 = TargetVelocity(dt);

                Vector3d target_acc = (target_vel2 - target_vel) / dt - vesselState.gravityForce;
                target_acc += vert_speed_ctrl.Compute(Vector3d.Dot(vesselState.surfaceVelocity.normalized, target_vel - vesselState.surfaceVelocity)) * vesselState.surfaceVelocity.normalized;
                target_acc += lateral_speed_ctrl.Compute(Vector3d.Exclude(vesselState.surfaceVelocity.normalized, target_vel - vesselState.surfaceVelocity));

                Vector3d dir = 0.1 * target_acc.normalized - vesselState.surfaceVelocity.normalized;
                core.attitude.attitudeTo(dir.normalized, AttitudeReference.INERTIAL, core.landing);

                double acc = Vector3d.Dot(target_acc, dir);

                core.thrust.tmode = MechJebModuleThrustController.TMode.DIRECT;
                core.thrust.trans_spd_act = Mathf.Clamp((float)((acc - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel) * 100), 0, 100);

                status += "\nVertical speed:\n" +
                    "  Target: " + Vector3d.Dot(vesselState.up, target_vel).ToString("F2") + " m/s\n" +
                    "  Actual: " + Vector3d.Dot(vesselState.up, vesselState.surfaceVelocity).ToString("F2") + " m/s";

                status += "\nHorizontal speed:\n" +
                    "  Target: " + Vector3d.Exclude(vesselState.up, target_vel).magnitude.ToString("F2") + " m/s\n" +
                    "  Actual: " + Vector3d.Exclude(vesselState.up, vesselState.surfaceVelocity).magnitude.ToString("F2") + " m/s";

                /*double target_vert_spd = TargetVelocity((float)minalt);
                double vert_spd = Vector3d.Dot(vesselState.surfaceVelocity, vesselState.up);
                double vert_acc = vert_speed_ctrl.Compute(target_vert_spd - vert_spd) + vesselState.gravityForce.magnitude;

                Vector3d target_acc;
                Vector3d target_vel;

                status += "\nVertical speed\n" + 
                    "  Target: " + target_vert_spd.ToString("F2") + " m/s\n" +
                    "  Actual: " + vert_spd.ToString("F2") + " m/s";

                if (core.landing.landAtTarget)
                {
                    Vector3d torque = vesselState.torqueAvailable + vesselState.torqueFromEngine * vessel.ctrlState.mainThrottle;
                    if (core.thrust.differentialThrottleSuccess)
                        torque += vesselState.torqueFromDiffThrottle * vessel.ctrlState.mainThrottle / 2;

                    Vector3d tmp = Vector3d.Scale(torque, vesselState.MoI.Invert());
                    double trq_over_moi = Math.Min(tmp.y, tmp.z);

                    double max_horiz_jerk = trq_over_moi * 2;
                    double max_horiz_vel = 50;

                    Vector3d target = Vector3d.Exclude(vesselState.up, core.target.RelativePosition);

                    double v = Math.Pow(4.5 * target.sqrMagnitude * max_horiz_jerk, 1.0 / 3.0);
                    if (v > max_horiz_vel)
                        v = max_horiz_vel;

                    target_vel = -v * target.normalized;

                    status += "\nHorizontal position error: " + target.magnitude.ToString("F2") + " m";
                    status += "\nHorizontal speed target: " + target_vel.magnitude.ToString("F2") + " m/s";
                }
                else
                {
                    target_vel = Vector3d.zero;
                }

                target_acc = horiz_speed_ctrl.Compute(target_vel - Vector3d.Exclude(vesselState.up, vesselState.surfaceVelocity))
                    + vesselState.up * vert_acc;

                core.attitude.attitudeTo(target_acc, AttitudeReference.INERTIAL, core.landing);

                core.thrust.tmode = MechJebModuleThrustController.TMode.DIRECT;
                core.thrust.trans_spd_act = Mathf.Clamp((float)((target_acc.magnitude - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel) * 100), 0, 100);

                horiz_speed_ctrl.intAccum = Vector3d.Exclude(vesselState.up, horiz_speed_ctrl.intAccum);
*/
                return this;
            }
        }
    }
}
