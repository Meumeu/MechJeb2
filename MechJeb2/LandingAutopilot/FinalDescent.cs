using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class FinalDescent : AutopilotStep
        {
            PIDController vert_speed_ctrl = new PIDController(2, 1, 0);
            PIDControllerV lateral_pos_ctrl = new PIDControllerV(0.1, 0, 0.4);

            List<TrajectoryPoint> descent_profile = new List<TrajectoryPoint>();
            readonly ReferenceFrame frame;

            public FinalDescent(MechJebCore core, List<TrajectoryPoint> trajectory, ReferenceFrame frame) : base(core)
            {
                if (core.landing.landAtTarget)
                {
                    Vector3d target = mainBody.GetRelSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0);
                    Vector3d landing_site = mainBody.GetRelSurfacePosition(trajectory.Last().pos.latitude, trajectory.Last().pos.longitude, 0);

                    Vector3d adjustment = target - landing_site;
                    double UT_start = trajectory.First().UT + 3;
                    double adjustment_duration = trajectory.Last().UT - 3 - UT_start;

                    Debug.Log("FinalDescent: adjustment of " + adjustment.magnitude.ToString("F2") + " m");
                    Debug.Log("Target: " + Coordinates.ToStringDMS(core.target.targetLatitude, core.target.targetLongitude));

                    double now = Planetarium.GetUniversalTime();
                    this.frame = new ReferenceFrame(mainBody);

                    foreach (TrajectoryPoint i in trajectory)
                    {
                        if (core.landing.landAtTarget)
                        {
                            // All vectors are relative to the celestial body
                            Vector3d pos = i.pos.ToRotatingAxesVelocity();
                            Vector3d svel = i.svel.ToRotatingAxesVelocity();

                            Vector3d tmp1 = adjustment * 2 * Math.PI / (adjustment_duration * adjustment_duration);
                            double tmp2 = 2 * Math.PI / adjustment_duration;

                            double t = Math.Min(adjustment_duration, Math.Max(0, i.UT - UT_start));

                            TrajectoryPoint pt;
                            Vector3d dpos = tmp1 / tmp2 * t - tmp1 / (tmp2 * tmp2) * Math.Sin(tmp2 * t);
                            svel += tmp1 / tmp2 * (1 - Math.Cos(tmp2 * t));

                            pt.pos = this.frame.ToAbsolute(pos + dpos, now);
                            pt.svel = this.frame.ToAbsolute(svel, now);
                            pt.UT = i.UT;
                            descent_profile.Add(pt);

                            Debug.Log(string.Format("Point #{0}: {1}, prev: {6}, alt {2:F2} m, vertical vel: {3:F2} m/s, dist to target: {4:F2} m, dpos: {5:F2} m",
                                descent_profile.Count,
                                Coordinates.ToStringDMS(pt.pos.latitude, pt.pos.longitude),
                                pt.pos.radius - mainBody.Radius,
                                Vector3d.Dot(pos.normalized, i.svel.ToRotatingAxesVelocity()),
                                Vector3d.Exclude(landing_site, landing_site - pos).magnitude,
                                dpos.magnitude,
                                Coordinates.ToStringDMS(i.pos.latitude, i.pos.longitude)
                            ));
                        }
                    }
                }
                else
                {
                    this.frame = frame;
                    foreach (TrajectoryPoint i in trajectory)
                    {
                        descent_profile.Add(i);

                        Debug.Log(string.Format("Point #{0}: {1}, alt {2:F2} m, vertical vel: {3:F2} m/s",
                            descent_profile.Count,
                            Coordinates.ToStringDMS(i.pos.latitude, i.pos.longitude),
                            i.pos.radius - mainBody.Radius,
                            Vector3d.Dot(vesselState.up, i.svel.ToInertialVelocity())));
                    }
                }

                core.warp.MinimumWarp(true);
                core.rcs.enabled = false;
            }

            // Finds the target acceleration, speed, position
            void Guidance(out Vector3d pos, out Vector3d svel, out Vector3d accel, out double timeToLand, out double altitude)
            {
                double terrainRadius = vesselState.altitudeBottom < 1000 ? mainBody.Radius + vesselState.surfaceAltitudeASL : mainBody.Radius + core.landing.LandingAltitude;
                altitude = vesselState.radiusBottom - terrainRadius;
                double radius = altitude + descent_profile[descent_profile.Count - 1].pos.radius;

                int i1, i2;
                double lambda;

                if (radius > descent_profile[0].pos.radius)
                {
                    i1 = 0;
                    i2 = 1;
                    lambda = 0;
                }
                else if (radius < descent_profile[descent_profile.Count - 1].pos.radius)
                {
                    i1 = descent_profile.Count - 2;
                    i2 = descent_profile.Count - 1;
                    lambda = 1;
                }
                else
                {
                    i1 = 0;
                    i2 = descent_profile.Count - 1;
                    while (i2 - i1 > 1)
                    {
                        int i = (i1 + i2) / 2;

                        if (descent_profile[i].pos.radius > radius)
                            i1 = i;
                        else
                            i2 = i;
                    }

                    lambda = (radius - descent_profile[i1].pos.radius) / (descent_profile[i2].pos.radius - descent_profile[i1].pos.radius);
                }

                Vector3d v1 = descent_profile[i1].svel.ToRotatingAxesVelocity();
                Vector3d v2 = descent_profile[i2].svel.ToRotatingAxesVelocity();

                Vector3d x1 = descent_profile[i1].pos.ToRotatingAxesPosition();
                Vector3d x2 = descent_profile[i2].pos.ToRotatingAxesPosition();

                double t1 = descent_profile[i1].UT;
                double t2 = descent_profile[i2].UT;

                pos = x1 + lambda * (x2 - x1);
                svel = v1 + lambda * (v2 - v1);
                accel = (v2 - v1) / (t2 - t1);
                timeToLand = descent_profile[descent_profile.Count - 1].UT - (t1 + lambda * (t2 - t1));


                if (Vector3d.Dot(vesselState.up, svel) > -core.landing.touchdownSpeed)
                {
                    svel = Vector3d.Exclude(vesselState.up, svel) - vesselState.up * core.landing.touchdownSpeed;
                    accel = Vector3d.Exclude(vesselState.up, accel);
                }
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

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (vessel.LandedOrSplashed)
                {
                    core.landing.StopLanding();
                    return null;
                }

                // TODO perhaps we should pop the parachutes at this point, or at least consider it depending on the altitude.
                Vector3d pos, svel, accel;
                double timeToLand, altitude;
                Guidance(out pos, out svel, out accel, out timeToLand, out altitude);
                status = "Final descent: " + altitude.ToString("F0") + "m above terrain\n";
                status += "Landing in " + timeToLand.ToString("F1") + "s";

                // transition smoothly from SVEL- to UP
                // FIXME: behave correctly if the vertical surface velocity is > 10 m/s
                Vector3d vertical = (-altitude * vesselState.surfaceVelocity.normalized + 100 * vesselState.up).normalized;

                Vector3d target_acc = accel - vesselState.gravityForce;
                Vector3d svel_err = svel - vesselState.surfaceVelocity;
                Vector3d pos_err = Vector3d.Exclude(vertical, pos - vesselState.positionBottom);

                // vertical control
                double vert_accel = vert_speed_ctrl.Compute(Vector3d.Dot(vertical, svel_err));
                // anti windup
                float max_integral = (float)(mainBody.GeeASL / vert_speed_ctrl.Ki);
                vert_speed_ctrl.intAccum = Mathf.Clamp((float)vert_speed_ctrl.intAccum, -max_integral, max_integral);
                target_acc += vert_accel * vertical;

                // horizontal control
                if (core.landing.landAtTarget)
                {
                    Vector3d horiz_accel = lateral_pos_ctrl.Compute(Vector3d.Exclude(vertical, pos_err));
                    target_acc += horiz_accel;
                    target_acc = LimitLateralAcceleration(target_acc, vertical);
                }


                if (core.landing.useRCS)
                {
                    core.rcs.enabled = true;
                    double acc = Vector3d.Dot(target_acc, vesselState.forward);
                    core.thrust.targetThrottle = Mathf.Clamp01((float)((acc - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel)));
                    core.rcs.SetWorldAcceleration(Vector3d.Exclude(vesselState.forward, target_acc));
                    core.attitude.attitudeTo(vertical, AttitudeReference.INERTIAL, core.landing);
                }
                else
                {
                    double acc = target_acc.magnitude;
                    core.thrust.targetThrottle = Mathf.Clamp01((float)((acc - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel)));
                    core.attitude.attitudeTo(target_acc.normalized, AttitudeReference.INERTIAL, core.landing);
                }

                status += "\nVertical speed:\n" +
                    "  Target: " + Vector3d.Dot(vesselState.up, svel).ToString("F2") + " m/s\n" +
                    "  Actual: " + Vector3d.Dot(vesselState.up, vesselState.surfaceVelocity).ToString("F2") + " m/s";

                status += "\nHorizontal position error: " + pos_err.magnitude.ToString("F2") + " m";

                status += "\nThrottle: " + (core.thrust.targetThrottle * 100).ToString("F1") + "%";

                return this;
            }
        }
    }
}
