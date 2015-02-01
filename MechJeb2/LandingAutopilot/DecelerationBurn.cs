using System;
using System.Linq;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class DecelerationBurn : AutopilotStep
        {
            public DecelerationBurn(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep OnFixedUpdate()
            {
                if (vesselState.altitudeASL < core.landing.DecelerationEndAltitude() + 5)
                {
                    core.warp.MinimumWarp();

                    if (core.landing.UseAtmosphereToBrake())
                        return new FinalDescent(core);
                    else
                        return new KillHorizontalVelocity(core);
                }

                //FIXME: warp to deceleration burn

                Vector3d desiredThrustVector = -vesselState.surfaceVelocity.normalized;

                Vector3d courseCorrection = core.landing.ComputeCourseCorrection(false);
                double correctionAngle = courseCorrection.magnitude / (2.0 * vesselState.limitedMaxThrustAccel);
                correctionAngle = Math.Min(0.1, correctionAngle);
                desiredThrustVector = (desiredThrustVector + correctionAngle * courseCorrection.normalized).normalized;

                if (Vector3d.Dot(vesselState.surfaceVelocity, vesselState.up) > 0
                    || Vector3d.Dot(vesselState.forward, desiredThrustVector) < 0.75)
                {
                    core.thrust.targetThrottle = 0;
                    status = "Braking";
                }
                else
                {
                    double controlledSpeed = vesselState.speedSurface * Math.Sign(Vector3d.Dot(vesselState.surfaceVelocity, vesselState.up)); //positive if we are ascending, negative if descending
                    double desiredSpeed = -core.landing.MaxAllowedSpeed();
                    double desiredSpeedAfterDt = -core.landing.MaxAllowedSpeedAfterDt(vesselState.deltaT);
                    double minAccel = -vesselState.localg * Math.Abs(Vector3d.Dot(vesselState.surfaceVelocity.normalized, vesselState.up));
                    double maxAccel = vesselState.maxThrustAccel * Vector3d.Dot(vesselState.forward, -vesselState.surfaceVelocity.normalized) - vesselState.localg * Math.Abs(Vector3d.Dot(vesselState.surfaceVelocity.normalized, vesselState.up));
                    const double speedCorrectionTimeConstant = 0.3;
                    double speedError = desiredSpeed - controlledSpeed;
                    double desiredAccel = speedError / speedCorrectionTimeConstant + (desiredSpeedAfterDt - desiredSpeed) / vesselState.deltaT;
                    if (maxAccel - minAccel > 0) core.thrust.targetThrottle = Mathf.Clamp((float)((desiredAccel - minAccel) / (maxAccel - minAccel)), 0.0F, 1.0F);
                    else core.thrust.targetThrottle = 0;
                    status = "Braking: target speed = " + Math.Abs(desiredSpeed).ToString("F1") + " m/s";
                }

                core.attitude.attitudeTo(desiredThrustVector, AttitudeReference.INERTIAL, core.landing);

                return this;
            }
        }
    }
}
