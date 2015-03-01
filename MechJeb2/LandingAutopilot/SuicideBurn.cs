using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class SuicideBurn : AutopilotStep
        {
            PIDController vert_speed_ctrl = new PIDController(2, 1, 0);
            PIDControllerV lateral_pos_ctrl = new PIDControllerV(0.1, 0, 0.4);

            List<TrajectoryPoint> descent_profile = new List<TrajectoryPoint>();
            readonly ReferenceFrame frame;

            double burnUt;
            bool suiciding; // FIXME: comment

            public SuicideBurn(MechJebCore core, List<TrajectoryPoint> trajectory, ReferenceFrame frame) : base(core)
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

                burnUt = core.landing.BurnUt;
                suiciding = false;

                core.warp.MinimumWarp(true);
                core.rcs.enabled = false;
            }

            double LandingRadius()
            {
                Vector3d landingSite = vesselState.altitudeBottom < 100 ? vesselState.CoM : mainBody.position + descent_profile[descent_profile.Count - 1].pos.ToRotatingAxesPosition();
                Vector3d up = (landingSite - mainBody.position).normalized;
                RaycastHit hit;

                if (Physics.Raycast(landingSite + 1000 * up, -up, out hit, Mathf.Infinity, 1 << 15))
                {
                    return Math.Max(mainBody.Radius, (hit.point - mainBody.position).magnitude);
                }
                else
                {
//                    return mainBody.TerrainAltitude(landingSite) + mainBody.Radius;
                    return descent_profile[descent_profile.Count - 1].pos.radius;
                }
            }

            int descent_profile_idx = 0;
            // Finds the target acceleration, speed, position
            void Guidance(out Vector3d pos, out Vector3d svel, out Vector3d accel, out double timeToLand, out double altitude)
            {
                double terrainRadius = LandingRadius();
                double altAdjustment = descent_profile[descent_profile.Count - 1].pos.radius - terrainRadius;

                altitude = vesselState.radiusBottom - terrainRadius;

                Vector3d currentPosition = vesselState.positionBottom + vesselState.up * altAdjustment;
                double lambda;

                while (descent_profile_idx < descent_profile.Count - 1)
                {
                    Vector3d x1 = descent_profile[descent_profile_idx].pos.ToRotatingAxesPosition();
                    Vector3d x2 = descent_profile[descent_profile_idx+1].pos.ToRotatingAxesPosition();
                    lambda = Vector3d.Dot(currentPosition - x1, x2 - x1) / (x2 - x1).sqrMagnitude;

                    if (lambda > 1)
                        descent_profile_idx++;
                    else
                        break;
                }

                if (descent_profile_idx == descent_profile.Count - 1)
                {
                    timeToLand = 0;
                    pos = vesselState.CoM - mainBody.position;
                    svel = -core.landing.touchdownSpeed * vesselState.up;
                    accel = Vector3d.zero;
                }
                else
                {
                    int i1 = descent_profile_idx;
                    int i2 = descent_profile_idx + 1;

                    Vector3d v1 = descent_profile[i1].svel.ToRotatingAxesVelocity();
                    Vector3d v2 = descent_profile[i2].svel.ToRotatingAxesVelocity();

                    Vector3d x1 = descent_profile[i1].pos.ToRotatingAxesPosition();
                    Vector3d x2 = descent_profile[i2].pos.ToRotatingAxesPosition();

                    double t1 = descent_profile[i1].UT;
                    double t2 = descent_profile[i2].UT;

                    pos = x1 + lambda * (x2 - x1);
                    svel = v1 + lambda * (v2 - v1);
                    accel = (v2 - v1) / (t2 - t1);
                    double t = t1 + lambda * (t2 - t1);

                    timeToLand = descent_profile[descent_profile.Count - 1].UT - t;

                    if (!suiciding && t > burnUt)
                    {
                        suiciding = true;
                        vert_speed_ctrl.intAccum = 0.95 * vesselState.maxThrustAccel / vert_speed_ctrl.Ki;
                    }
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

            public override AutopilotStep OnFixedUpdate()
            {
                if (vessel.LandedOrSplashed)
                {
                    core.landing.StopLanding();
                    return null;
                }

                if (Vector3d.Dot(vesselState.up, vesselState.surfaceVelocity) > -core.landing.touchdownSpeed)
                    return new Touchdown(core);

                // TODO perhaps we should pop the parachutes at this point, or at least consider it depending on the altitude.

                // Guidance
                Vector3d pos, svel, accel;
                double timeToLand, altitude;
                Guidance(out pos, out svel, out accel, out timeToLand, out altitude);
                Vector3d dir = MechJebModuleLandingAutopilot.ThrustDirection(vesselState.CoM - mainBody.position, vesselState.surfaceVelocity);

                status = "Suicide burn: " + altitude.ToString("F0") + "m above terrain\n";
                status += "Landing in " + timeToLand.ToString("F1") + "s";


                // Compute the control error
                Vector3d svel_err = svel - vesselState.surfaceVelocity;
                Vector3d pos_err = pos - vesselState.CoM;

                status += "\nVertical speed:\n" +
                    "  Target: " + Vector3d.Dot(vesselState.up, svel).ToString("F2") + " m/s\n" +
                    "  Actual: " + Vector3d.Dot(vesselState.up, vesselState.surfaceVelocity).ToString("F2") + " m/s";

                status += "\nPosition error: " + pos_err.magnitude.ToString("F2") + " m";


                // vertical control
                double vert_accel = vert_speed_ctrl.Compute(Vector3d.Dot(dir, svel_err));
                // anti windup
//                float max_integral = (float)(mainBody.GeeASL / vert_speed_ctrl.Ki);
//                vert_speed_ctrl.intAccum = Mathf.Clamp((float)vert_speed_ctrl.intAccum, -max_integral, max_integral);
                //target_acc += vert_accel * dir;
                Vector3d target_acc = vert_accel * dir;
//                double minVertAccel = vesselState.gravityForce.magnitude / 2;
                double minVertAccel = 0;
                if (Vector3d.Dot(target_acc, vesselState.up) < minVertAccel)
                    target_acc = Vector3d.Exclude(vesselState.up, target_acc) + minVertAccel * vesselState.up;

                // horizontal control
                if (core.landing.landAtTarget)
                {
//                    Vector3d horiz_accel;
//                    horiz_accel = lateral_pos_ctrl.Compute(Vector3d.Exclude(vertical, pos_err));
//                    target_acc += horiz_accel;
//                    target_acc = LimitLateralAcceleration(target_acc, vertical);
                }

                if (core.landing.useRCS)
                {
//                    core.rcs.enabled = true;
//                    double acc = Vector3d.Dot(target_acc, vertical);
//                    core.thrust.targetThrottle = Mathf.Clamp01((float)((acc - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel)));
//                    core.rcs.SetWorldAcceleration(Vector3d.Exclude(vertical, target_acc));
//                    core.attitude.attitudeTo(vertical, AttitudeReference.INERTIAL, core.landing);

                    double acc = target_acc.magnitude;
                    core.thrust.targetThrottle = Mathf.Clamp01((float)((acc - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel)));
                    core.attitude.attitudeTo(dir, AttitudeReference.INERTIAL, core.landing);
                }
                else
                {
                    double acc = target_acc.magnitude;
                    core.thrust.targetThrottle = Mathf.Clamp01((float)((acc - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel)));
                    core.attitude.attitudeTo(dir, AttitudeReference.INERTIAL, core.landing);
                }


                status += "\nThrottle: " + (core.thrust.targetThrottle * 100).ToString("F1") + "%";
                status += "\nIntegral term: " + (vert_speed_ctrl.intAccum * vert_speed_ctrl.Ki).ToString("F1") + " m/s^2";

                return this;
            }
        }
    }
}
