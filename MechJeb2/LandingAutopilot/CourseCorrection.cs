using System;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class CourseCorrection : AutopilotStep
        {
            bool courseCorrectionBurning = false;

            public CourseCorrection(MechJebCore core) : base(core)
            {
                core.warp.MinimumWarp();
                core.landing.maxSimulationAge = 0;
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (!core.landing.PredictionReady)
                    return this;

                // If the atomospheric drag is at least 100mm/s2 then start trying to target the overshoot using the parachutes
                if (mainBody.DragAccel(vesselState.CoM, vesselState.orbitalVelocity, vesselState.massDrag / vesselState.mass).magnitude > 0.1)
                {
                    if (core.landing.ParachutesDeployable())
                    {
						//FIXME: use parachutes
                    }
                }

                LandedReentryResult landed = core.landing.prediction as LandedReentryResult;
                double currentError = Vector3d.Distance(mainBody.GetRelSurfacePosition(landed.landingSite.latitude, landed.landingSite.longitude, 0),
                    mainBody.GetRelSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0));

                Vector3d deltaV = core.landing.ComputeCourseCorrection(true);

                double timeToDecelerationBurn = (core.landing.BurnUt - 2) - Planetarium.GetUniversalTime();

                // If a parachute has already been deployed then we will not be able to control attitude anyway, so move back to the coast to deceleration step.
                if ((currentError < 150 && deltaV.magnitude < 2) ||
                    vesselState.parachuteDeployed ||
                    (core.landing.LandingBurnReady && timeToDecelerationBurn < 10))
                {
                    core.thrust.targetThrottle = 0;
                    core.landing.maxSimulationAge = MechJebModuleLandingAutopilot.defaultMaxSimulationAge;
                    return new CoastToDeceleration(core);
                }


                status = "Performing course correction of about " + deltaV.magnitude.ToString("F1") + " m/s";

                core.attitude.attitudeTo(deltaV.normalized, AttitudeReference.INERTIAL, core.landing);

                if (core.attitude.attitudeAngleFromTarget() < 5)
                    courseCorrectionBurning = true;
                else if (core.attitude.attitudeAngleFromTarget() > 30)
                    courseCorrectionBurning = false;

                if (courseCorrectionBurning)
                {
                    const double timeConstant = 2.0;
                    core.thrust.ThrustForDV(deltaV.magnitude, timeConstant);
                    deltaV -= vesselState.thrustVectorLastFrame * TimeWarp.fixedDeltaTime;
                }
                else
                {
                    core.thrust.targetThrottle = 0;
                }

                return this;
            }
        }
    }
}
