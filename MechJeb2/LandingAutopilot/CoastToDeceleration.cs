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

                status = "Coasting toward deceleration burn";

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

                        if (deltaV.magnitude > 3)
                            core.rcs.enabled = true;
                        else if (deltaV.magnitude < 0.1)
                            core.rcs.enabled = false;

                        if (core.rcs.enabled)
                        {
                            warpReadyCourseCorrection = false;
                            core.rcs.SetWorldVelocityError(deltaV);
                        }
                        else
                        {
                            warpReadyCourseCorrection = true;
                        }

                        status += "\nCourse correction DV: " + deltaV.magnitude.ToString("F3") + " m/s";
                    }
                }

                // Transition into the final decent step 5s before the suicide burn
                if (!double.IsNaN(core.landing.BurnUt) && core.landing.BurnUt < Planetarium.GetUniversalTime() + 5)
                {
                    core.warp.MinimumWarp();
                    return new FinalDescent(core,
                        (core.landing.prediction as LandedReentryResult).trajectory,
                        (core.landing.prediction as LandedReentryResult).frame);
                }

                if (core.attitude.attitudeAngleFromTarget() < 1) { warpReadyAttitudeControl = true; } // less warp start warp stop jumping
                if (core.attitude.attitudeAngleFromTarget() > 10) { warpReadyAttitudeControl = false; } // hopefully

                core.attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, core.landing);

                // Warp at a rate no higher than the rate that would have us start the burn 10 seconds from now:
                if (core.node.autowarp)
                {
                    if (warpReadyAttitudeControl && warpReadyCourseCorrection && !double.IsNaN(core.landing.BurnUt))
                        core.warp.WarpRegularAtRate((float)(core.landing.BurnUt - Planetarium.GetUniversalTime()) / 10);
                    else
                        core.warp.MinimumWarp();
                }

                return this;
            }
        }
    }
}
