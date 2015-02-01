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
            PIDControllerV horiz_speed_ctrl = new PIDControllerV(0.1, 0, 0.4);

            List<KeyValuePair<float, float>> vertical_velocity_profile;

            public FinalDescent(MechJebCore core) : base(core)
            {
                // TODO: compute a vertical velocity profile
                vertical_velocity_profile = new List<KeyValuePair<float, float>>();

                double vertical_accel = vesselState.limitedMaxThrustAccel * 0.9 - mainBody.GeeASL * 9.81;
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
                    vertical_velocity_profile.Add(new KeyValuePair<float, float>(0, 0));
                }

                core.warp.MinimumWarp(true);
            }

            float TargetVelocity(float alt)
            {
                if (alt > vertical_velocity_profile[0].Key)
                    return vertical_velocity_profile[0].Value;

                if (alt < vertical_velocity_profile[vertical_velocity_profile.Count - 1].Key)
                    return vertical_velocity_profile[vertical_velocity_profile.Count - 1].Value;

                for (int i = 1; i < vertical_velocity_profile.Count; i++)
                {
                    if ((alt < vertical_velocity_profile[i - 1].Key) &&
                        (alt >= vertical_velocity_profile[i].Key))
                    {
                        float dh = vertical_velocity_profile[i].Key - vertical_velocity_profile[i - 1].Key;
                        float dv = vertical_velocity_profile[i].Value - vertical_velocity_profile[i - 1].Value;
                        return vertical_velocity_profile[i - 1].Value + (alt - vertical_velocity_profile[i - 1].Key) * dv / dh;
                    }
                }

                throw new ArgumentOutOfRangeException();
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (vessel.LandedOrSplashed)
                {
                    core.landing.StopLanding();
                    return null;
                }


                // TODO perhaps we should pop the parachutes at this point, or at least consider it depending on the altitude.

                // TODO: use altitudeASL once we have an actual vertical velocity profile
                double minalt = Math.Min(vesselState.altitudeBottom, Math.Min(vesselState.altitudeASL, vesselState.altitudeTrue));
                status = "Final descent: " + minalt.ToString("F0") + "m above terrain";

                /*if (vesselState.limitedMaxThrustAccel < vesselState.gravityForce.magnitude)
                {
                    // if we have TWR < 1, just try as hard as we can to decelerate:
                    // (we need this special case because otherwise the calculations spit out NaN's)
                    core.thrust.tmode = MechJebModuleThrustController.TMode.KEEP_VERTICAL;
                    core.thrust.trans_kill_h = true;
                    core.thrust.trans_spd_act = 0;
                }
                else*/
                {
                    double target_vert_spd = TargetVelocity((float)minalt);
                    double vert_spd = Vector3d.Dot(vesselState.surfaceVelocity, vesselState.up);
                    double vert_acc = vert_speed_ctrl.Compute(target_vert_spd - vert_spd) + vesselState.gravityForce.magnitude;

                    Vector3d target_acc;

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

                        Vector3d target_vel = -v * target.normalized;
                        target_acc = Vector3d.Exclude(vesselState.up,
                            horiz_speed_ctrl.Compute(target_vel - Vector3d.Exclude(vesselState.up, vesselState.surfaceVelocity)))
                            + vesselState.up * vert_acc;

                        status += "\nHorizontal position error: " + target.magnitude.ToString("F2") + " m";
                        status += "\nHorizontal speed target: " + target_vel.magnitude.ToString("F2") + " m/s";

                        core.attitude.attitudeTo(target_acc, AttitudeReference.INERTIAL, core.landing);
                    }
                    else
                    {
                        target_acc = vesselState.up * vert_acc;
                        core.attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, core.landing);
                    }

                    core.thrust.tmode = MechJebModuleThrustController.TMode.DIRECT;
                    core.thrust.trans_spd_act = Mathf.Clamp((float)((target_acc.magnitude - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel) * 100), 0, 100);

                }

                return this;
            }
        }
    }
}
