using UnityEngine;
using System;

namespace TrafficSimulation
{
    [Serializable]
    public enum DriveType
    {
        RearWheelDrive,
        FrontWheelDrive,
        AllWheelDrive
    }

    [Serializable]
    public enum UnitType
    {
        KMH,
        MPH
    }

    public class WheelDrive : MonoBehaviour
    {
        [Tooltip("Downforce applied to the vehicle")]
        public float downForce = 100f;

        [Tooltip("Maximum steering angle of the wheels")] //макс кут повороту коліс
        public float maxAngle = 30f;

        [Tooltip("Speed at which we will reach the above steering angle (lerp)")] //Швидкість, з якою ми досягнемо вищевказаного кута повороту
        public float steeringLerp = 5f;

        [Tooltip("Max speed (in unit choosen below) when the vehicle is about to steer")]//макc швидкість коли збирається повертати
        public float steeringSpeedMax = 20f;

        [Tooltip("Maximum torque applied to the driving wheels")] //Максимальний крутний момент, що застосовується до ведучих коліс
        public float maxTorque = 300f;

        [Tooltip("Maximum brake torque applied to the driving wheels")] //гальмівний момент
        public float brakeTorque = 30000f;

        [Tooltip("Unit Type")]
        public UnitType unitType;

        [Tooltip("Min Speed - when driving (not including stops/brakes), in the unit choosen above. Should be > 0.")] //Мінімальна швидкість - під час руху (без урахування зупинок / гальм)
        public float minSpeed = 5;

        [Tooltip("Max Speed in the unit choosen above")] //Максимальна швидкість у вибраному пристрої вище
        public float maxSpeed = 50;

        [Tooltip("Drag the wheel shape here.")] // фопрма колеса
        public GameObject leftWheelShape;
        public GameObject rightWheelShape;

        [Tooltip("Whether you want to animate the wheels")]//анімація коліс
        public bool animateWheels = true;

        [Tooltip("The vehicle's drive type: rear-wheels drive, front-wheels drive or all-wheels drive.")] //Тип приводу автомобіля: привід на задні колеса, привід на передні колеса або повний привід.
        public DriveType driveType;

        private WheelCollider[] wheels;
        private float currentSteering = 0f;

        void OnEnable()
        {
            wheels = GetComponentsInChildren<WheelCollider>();

            for (int i = 0; i < wheels.Length; ++i)
            {
                var wheel = wheels[i];

                // Create wheel shapes only when needed. створення форм коліс коли потрібно
                if (leftWheelShape != null && wheel.transform.localPosition.x < 0)
                {
                    var ws = Instantiate(leftWheelShape);
                    ws.transform.parent = wheel.transform;
                }
                else if (rightWheelShape != null && wheel.transform.localPosition.x > 0)
                {
                    var ws = Instantiate(rightWheelShape);
                    ws.transform.parent = wheel.transform;
                }

                wheel.ConfigureVehicleSubsteps(10, 1, 1);
            }
        }

        public void Move(float _acceleration, float _steering, float _brake) //рух
        {

            float nSteering = Mathf.Lerp(currentSteering, _steering, Time.deltaTime * steeringLerp);
            currentSteering = nSteering;

            Rigidbody rb = this.GetComponent<Rigidbody>();

            float angle = maxAngle * nSteering; //визначаємо кут повороту
            float torque = maxTorque * _acceleration; //визначаємо крутий момент

            float handBrake = _brake > 0 ? brakeTorque : 0;

            foreach (WheelCollider wheel in wheels)
            {
                // Steer front wheels only
                if (wheel.transform.localPosition.z > 0) wheel.steerAngle = angle;

                if (wheel.transform.localPosition.z < 0) wheel.brakeTorque = handBrake;

                if (wheel.transform.localPosition.z < 0 && driveType != DriveType.FrontWheelDrive) wheel.motorTorque = torque;

                if (wheel.transform.localPosition.z >= 0 && driveType != DriveType.RearWheelDrive) wheel.motorTorque = torque;


                // Update visual wheels if allowed
                if (animateWheels)
                {
                    Quaternion q;
                    Vector3 p;
                    wheel.GetWorldPose(out p, out q);

                    Transform shapeTransform = wheel.transform.GetChild(0);
                    shapeTransform.position = p;
                    shapeTransform.rotation = q;
                }
            }


            //Apply speed
            float s = GetSpeedUnit(rb.velocity.magnitude);
            if (s > maxSpeed) rb.velocity = GetSpeedMS(maxSpeed) * rb.velocity.normalized;


            //Apply downforce
            rb.AddForce(-transform.up * downForce * rb.velocity.magnitude);
        }

        public float GetSpeedMS(float _s)
        {
            return unitType == UnitType.KMH ? _s / 3.6f : _s / 2.237f;
        }

        public float GetSpeedUnit(float _s)
        {
            return unitType == UnitType.KMH ? _s * 3.6f : _s * 2.237f;
        }
    }
}
