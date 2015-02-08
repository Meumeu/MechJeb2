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
            PIDController vert_pos_ctrl = new PIDController(2, 1, 0);
            PIDControllerV lateral_speed_ctrl = new PIDControllerV(0.1, 0, 0.4);
            PIDControllerV lateral_pos_ctrl = new PIDControllerV(0.1, 0, 0.4);

            List<TrajectoryPoint> descent_profile = new List<TrajectoryPoint>();
            readonly ReferenceFrame frame;

            public FinalDescent(MechJebCore core, List<TrajectoryPoint> trajectory, ReferenceFrame frame) : base(core)
            {
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
                    descent_profile.Add(i);
                    Debug.Log(string.Format("Point #{0}: {1}, alt {2} m", descent_profile.Count, Coordinates.ToStringDMS(i.pos.latitude, i.pos.longitude), i.pos.radius - mainBody.Radius));
                }

                core.warp.MinimumWarp(true);
            }

            // Finds the target acceleration, speed, position for a given time
            void Guidance(out Vector3d pos, out Vector3d svel, out Vector3d accel)
            {
                /*if (UT < descent_profile[0].UT)
                {
                    pos = frame.WorldPositionAtCurrentTime(descent_profile[0].pos);
                    svel = frame.WorldVelocityAtCurrentTime(descent_profile[0].svel);
                    return;
                }

                if (UT > descent_profile[descent_profile.Count - 1].UT)
                {
                    svel = frame.WorldVelocityAtCurrentTime(descent_profile[descent_profile.Count - 1].svel);
                    if (Vector3d.Dot(vesselState.up, svel) > -core.landing.touchdownSpeed)
                        svel = Vector3d.Exclude(vesselState.up, svel) - vesselState.up * core.landing.touchdownSpeed;

                    pos = frame.WorldPositionAtCurrentTime(descent_profile[descent_profile.Count - 1].pos)
                        + svel * (UT - descent_profile[descent_profile.Count - 1].UT);

                    return;
                }*/

                //double radius = vesselState.radius;
                double radius = vesselState.radiusBottom;

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
                    /*Vector3d pos1 = frame.WorldPositionAtCurrentTime(descent_profile[i1].pos);
                    Vector3d pos2 = frame.WorldPositionAtCurrentTime(descent_profile[i2].pos);

                    int i = (i1 + i2) / 2;

                    Vector3d pos3 = frame.WorldPositionAtCurrentTime(descent_profile[i].pos);

                    if (Vector3d.Dot(pos3 - pos1, pos2 - pos1) > Vector3d.Dot(current_pos - pos1, pos2 - pos1))
                        i2 = i;
                    else
                        i1 = i;*/

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

                //double lambda = Vector3d.Dot(current_pos - x1, x2 - x1) / (x2 - x1).sqrMagnitude;
                double lambda = (radius - descent_profile[i1].pos.radius) / (descent_profile[i2].pos.radius - descent_profile[i1].pos.radius);

                pos = x1 + lambda * (x2 - x1);
                svel = v1 + lambda * (v2 - v1);
                accel = (v2 - v1) / (t2 - t1);
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
                target_acc += vert_accel * vertical;

                // horizontal control
                Vector3d horiz_accel = lateral_speed_ctrl.Compute(Vector3d.Exclude(vertical, svel_err));
                if (horiz_accel.magnitude > 0.2 * target_acc.magnitude)
                    horiz_accel = 0.2 * horiz_accel.normalized * target_acc.magnitude;

                target_acc += horiz_accel;

//                Vector3d horiz_err = Vector3d.Exclude(vesselState.up, pos_err);

                //Vector3d svel_err = svel - vesselState.surfaceVelocity /*+ vert_pos_ctrl.Compute(alt_err) * vesselState.up + lateral_pos_ctrl.Compute(horiz_err)*/;



                //target_acc += vert_speed_ctrl.Compute(Vector3d.Dot(vesselState.surfaceVelocity.normalized, svel_err)) * vesselState.surfaceVelocity.normalized;
                //target_acc += lateral_speed_ctrl.Compute(Vector3d.Exclude(vesselState.surfaceVelocity.normalized, svel_err));
                //lateral_speed_ctrl.intAccum = Vector3d.Exclude(vesselState.up, lateral_speed_ctrl.intAccum);

                //Vector3d dir = 0.1 * target_acc.normalized - vesselState.surfaceVelocity.normalized;
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
