using System;
using System.Collections.Generic;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class FinalDescent : AutopilotStep
        {
            PIDController vert_speed_ctrl = new PIDController(2, 1, 0);
            PIDControllerV lateral_speed_ctrl = new PIDControllerV(0.1, 0, 0.4);
            PIDControllerV lateral_pos_ctrl = new PIDControllerV(0.1, 0, 0.4);

            List<TrajectoryPoint> descent_profile = new List<TrajectoryPoint>();
            readonly ReferenceFrame frame;

            public FinalDescent(MechJebCore core, List<TrajectoryPoint> trajectory, ReferenceFrame frame) : base(core)
            {
                this.frame = frame;

                foreach (var i in trajectory)
                {
                    descent_profile.Add(i);
                    Debug.Log(string.Format("Point #{0}: {1}, alt {2:F2} m, vertical vel: {3:F2} m/s",
                        descent_profile.Count,
                        Coordinates.ToStringDMS(i.pos.latitude, i.pos.longitude),
                        i.pos.radius - mainBody.Radius,
                        Vector3d.Dot(vesselState.up, frame.WorldVelocityAtCurrentTime(i.svel))
                    ));
                }

                core.warp.MinimumWarp(true);
            }

            // Finds the target acceleration, speed, position for a given time
            void Guidance(out Vector3d pos, out Vector3d svel, out Vector3d accel)
            {
                double radius = vesselState.altitudeBottom + descent_profile[descent_profile.Count - 1].pos.radius;

                if (radius > descent_profile[0].pos.radius)
                {
                    pos = frame.WorldPositionAtCurrentTime(descent_profile[0].pos);
                    svel = frame.WorldVelocityAtCurrentTime(descent_profile[0].svel);
                    accel = Vector3d.zero;
                    return;
                }

                if (radius < descent_profile[descent_profile.Count - 1].pos.radius)
                {
                    pos = frame.WorldPositionAtCurrentTime(descent_profile[descent_profile.Count - 1].pos);
                    svel = frame.WorldVelocityAtCurrentTime(descent_profile[descent_profile.Count - 1].svel);
                    accel = Vector3d.zero;
                    return;
                }

                int i1 = 0;
                int i2 = descent_profile.Count - 1;
                while (i2 - i1 > 1)
                {
                    int i = (i1 + i2) / 2;

                    if (descent_profile[i].pos.radius > radius)
                        i1 = i;
                    else
                        i2 = i;
                }

                Vector3d v1 = frame.WorldVelocityAtCurrentTime(descent_profile[i1].svel);
                Vector3d v2 = frame.WorldVelocityAtCurrentTime(descent_profile[i2].svel);

                Vector3d x1 = frame.WorldPositionAtCurrentTime(descent_profile[i1].pos);
                Vector3d x2 = frame.WorldPositionAtCurrentTime(descent_profile[i2].pos);

                double t1 = descent_profile[i1].UT;
                double t2 = descent_profile[i2].UT;

                double lambda = (radius - descent_profile[i1].pos.radius) / (descent_profile[i2].pos.radius - descent_profile[i1].pos.radius);

                pos = x1 + lambda * (x2 - x1);
                svel = v1 + lambda * (v2 - v1);
                accel = (v2 - v1) / (t2 - t1);
            }

            Vector3d LimitLateralAcceleration(Vector3d acc, Vector3d vertical)
            {
                Vector3d lat = Vector3d.Exclude(vertical, acc);
                double vert = Vector3d.Dot(vertical, acc);

                if (vert < 0)
                    return vertical * 1e-10;
                else if (lat.magnitude > 0.2 * vert)
                    return vert * vertical + lat.normalized * 0.2 * vert;
                else
                    return acc;
            }

            double last_vert_vel;
            double last_horiz_vel;

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (vessel.LandedOrSplashed)
                {
                    Debug.Log(String.Format("Touchdown: vertical velocity: {0:F2} m/s, horizontal velocity: {1:F2} m/s", last_vert_vel, last_horiz_vel));
                    core.landing.StopLanding();
                    return null;
                }

                last_vert_vel = Vector3d.Dot(vesselState.up, vesselState.surfaceVelocity);
                last_horiz_vel = Vector3d.Exclude(vesselState.up, vesselState.surfaceVelocity).magnitude;

                // TODO perhaps we should pop the parachutes at this point, or at least consider it depending on the altitude.

                double minalt = core.landing.MinAltitude();
                status = "Final descent: " + minalt.ToString("F0") + "m above terrain";

                Vector3d pos, svel, accel;
                Guidance(out pos, out svel, out accel);
                if (Vector3d.Dot(vesselState.up, svel) > -core.landing.touchdownSpeed)
                    svel = Vector3d.Exclude(vesselState.up, svel) - vesselState.up * core.landing.touchdownSpeed;

                Vector3d target_acc = accel - vesselState.gravityForce;
                Vector3d svel_err = svel - vesselState.surfaceVelocity;
                Vector3d pos_err = Vector3d.Exclude(svel, pos - vesselState.positionBottom);


                // transition smoothly from SVEL- to UP
                Vector3d vertical = (-vesselState.surfaceVelocity + 10 * vesselState.up).normalized;

                // vertical control
                double vert_accel = vert_speed_ctrl.Compute(Vector3d.Dot(vertical, svel_err));
                vert_speed_ctrl.intAccum = Mathf.Clamp((float)vert_speed_ctrl.intAccum, -(float)mainBody.GeeASL, (float)mainBody.GeeASL); // anti windup
                target_acc += vert_accel * vertical;

                // horizontal control
                Vector3d horiz_accel = lateral_speed_ctrl.Compute(Vector3d.Exclude(vertical, svel_err));
                target_acc += horiz_accel;

                target_acc = LimitLateralAcceleration(target_acc, vertical);

                core.attitude.attitudeTo(target_acc.normalized, AttitudeReference.INERTIAL, core.landing);

                double acc = target_acc.magnitude;

                core.thrust.tmode = MechJebModuleThrustController.TMode.DIRECT;
                core.thrust.trans_spd_act = Mathf.Clamp((float)((acc - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel) * 100), 0, 100);

                status += "\nVertical speed:\n" +
                    "  Target: " + Vector3d.Dot(vesselState.up, svel).ToString("F2") + " m/s\n" +
                    "  Actual: " + Vector3d.Dot(vesselState.up, vesselState.surfaceVelocity).ToString("F2") + " m/s";

                status += "\nHorizontal speed:\n" +
                    "  Target: " + Vector3d.Exclude(vesselState.up, svel).magnitude.ToString("F2") + " m/s\n" +
                    "  Actual: " + Vector3d.Exclude(vesselState.up, vesselState.surfaceVelocity).magnitude.ToString("F2") + " m/s\n" +
                    "  Error: " + Vector3d.Exclude(vesselState.up, svel - vesselState.surfaceVelocity).magnitude.ToString("F2") + " m/s";

                status += "\nLateral error: " + Vector3d.Exclude(vesselState.up, pos_err).magnitude.ToString("F2") + " m";
                status += "\nVertical error: " + Vector3d.Dot(vesselState.up, pos_err).ToString("F2") + " m";
                status += "\nThrottle: " + core.thrust.trans_spd_act.ToString("F1") + "%";

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
