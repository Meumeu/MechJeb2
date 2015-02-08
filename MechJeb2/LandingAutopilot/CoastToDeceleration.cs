using System;
using System.Linq;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class CoastToDeceleration : AutopilotStep
        {
            public CoastToDeceleration(MechJebCore core) : base(core)
            {
                if (core.landing.rcsCourseCorrection && core.landing.landAtTarget)
                    core.rcs.enabled = true;
            }

            bool warpReadyAttitudeControl = true;
            bool warpReadyCourseCorrection = true;

            public override AutopilotStep OnFixedUpdate()
            {
                core.thrust.targetThrottle = 0;

                // If the atmospheric drag is has started to act on the vessel then we are in a position to start considering when to deploy the parachutes.
                if (mainBody.DragAccel(vesselState.CoM, vesselState.orbitalVelocity, vesselState.massDrag / vesselState.mass).magnitude > 0.10)
                {
                    if (core.landing.ParachutesDeployable())
                    {
						//FIXME: use parachutes
                    }
                }

                double timeToDecelerationBurn = (core.landing.BurnUt - 2) - Planetarium.GetUniversalTime();

                // Transition into the final decent step 5s before the suicide burn
                if (!core.landing.LandingBurnReady)
                {
                    status = "Computing landing burn";
                }
                else if (timeToDecelerationBurn > 0)
                {
                    status = "Landing burn in " + GuiUtils.TimeToDHMS(timeToDecelerationBurn, 1);
                }
                else
                {
                    core.warp.MinimumWarp();
                    core.rcs.SetWorldVelocityError(Vector3d.zero);
                    return new FinalDescent(core,
                        (core.landing.prediction as LandedReentryResult).trajectory,
                        (core.landing.prediction as LandedReentryResult).frame);
                }

                // If there is already a parachute deployed, then do not bother trying
                // to correct the course as we will not have any attitude control anyway.
                if (core.landing.landAtTarget && core.landing.PredictionReady && !core.landing.ParachutesDeployed())
                {
                    double currentError = Vector3d.Distance(core.target.GetPositionTargetPosition(), core.landing.LandingSite);

                    if (currentError > 1000)
                    {
                        core.rcs.enabled = false;
                        return new CourseCorrection(core);
                    }
                    else if (core.landing.rcsCourseCorrection)
                    {
                        Vector3d deltaV = core.landing.ComputeCourseCorrection(true);

                        if (deltaV.magnitude > 3 || (core.landing.LandingBurnReady && timeToDecelerationBurn < 30 && deltaV.magnitude > 1))
                            core.rcs.enabled = true;
                        else if (deltaV.magnitude < 0.5)
                            core.rcs.enabled = false;

                        if (core.rcs.enabled)
                        {
                            core.landing.maxSimulationAge = 0;
                            warpReadyCourseCorrection = false;
                            core.rcs.SetWorldVelocityError(deltaV);
                        }
                        else
                        {
                            core.landing.maxSimulationAge = 2;
                            warpReadyCourseCorrection = true;
                        }

                        status += "\nCourse correction DV: " + deltaV.magnitude.ToString("F3") + " m/s";
                    }
                }

                if (core.attitude.attitudeAngleFromTarget() < 1) { warpReadyAttitudeControl = true; } // less warp start warp stop jumping
                if (core.attitude.attitudeAngleFromTarget() > 10) { warpReadyAttitudeControl = false; } // hopefully

                // Warp at a rate no higher than the rate that would have us start the burn 10 seconds from now:
                if (core.node.autowarp)
                {
                    if (vesselState.altitudeASL < mainBody.RealMaxAtmosphereAltitude())
                    {
                        core.attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, core.landing);
                    }
                    else if (core.landing.LandingBurnReady)
                    {
                        double atmosphereEntryUT = orbit.NextTimeOfRadius(Planetarium.GetUniversalTime(), mainBody.Radius + mainBody.RealMaxAtmosphereAltitude());
                        double warpEndUT = Math.Min(core.landing.BurnUt, atmosphereEntryUT);

                        Vector3d svel = orbit.SwappedOrbitalVelocityAtUT(warpEndUT) - mainBody.getRFrmVel(orbit.SwappedAbsolutePositionAtUT(warpEndUT));
                        core.attitude.attitudeTo(-svel.normalized, AttitudeReference.INERTIAL, core.landing);
                    }

                    if (warpReadyAttitudeControl && warpReadyCourseCorrection && core.landing.LandingBurnReady)
                        core.warp.WarpRegularAtRate((float)(core.landing.BurnUt - Planetarium.GetUniversalTime()) / 10);
                    else
                        core.warp.MinimumWarp();
                }
                else
                {
                    core.attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, core.landing);
                }

                return this;
            }
        }
    }
}
