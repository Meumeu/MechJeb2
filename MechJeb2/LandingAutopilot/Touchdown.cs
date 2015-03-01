using System;

namespace MuMech
{
    namespace Landing
    {
        public class Touchdown : AutopilotStep
        {
            public Touchdown(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (vessel.LandedOrSplashed)
                {
                    core.landing.StopLanding();
                    return null;
                }

                core.thrust.tmode = MechJebModuleThrustController.TMode.DIRECT;
                core.thrust.targetThrottle = (float)((vesselState.gravityForce.magnitude - vesselState.minThrustAccel) / (vesselState.maxThrustAccel - vesselState.minThrustAccel));

                core.attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, core.landing);
//                core.thrust.targetThrottle = 0;
                return this;
            }
        }
    }
}

