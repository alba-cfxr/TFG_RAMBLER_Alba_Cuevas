/******************************************************
 * Autora: Alba Cuevas Fernandez
 * Grado en Ingenieria Electronica, Robotica y Mecatronica
 * Universidad de Malaga
 * Trabajo de Fin de Grado: Modelado y Simulacion del Robot Movil RAMBLER
 * Año: 2026
 ******************************************************/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp.Control
{
    public class Robot_Controller_RAMBLER : MonoBehaviour
    {
        public string topicName = "/cmd_vel";

        public GameObject front_left_wheel;
        public GameObject front_right_wheel;
        public GameObject back_left_wheel;
        public GameObject back_right_wheel;

        private ArticulationBody wFL;
        private ArticulationBody wFR;
        private ArticulationBody wBL;
        private ArticulationBody wBR;

        public float maxLinearSpeed = 22.2f;      // m/s
        public float maxRotationalSpeed = 80f;
        public float wheelRadius = 0.50f;       // meters
        public float trackWidth = 1.05f;        // meters Distance between line of tyres
        public float forceLimit = 450;
        //public float damping = 450;

        private float sentidoVelocidad = 0;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        void StartRAMBLER()
        {
            wFL = front_left_wheel.GetComponent<ArticulationBody>();
            wFR = front_right_wheel.GetComponent<ArticulationBody>();
            wBL = back_left_wheel.GetComponent<ArticulationBody>();
            wBR = back_right_wheel.GetComponent<ArticulationBody>();

            

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmdRAMBLER);
        }

        void ReceiveROSCmdRAMBLER(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
        }

        void FixedUpdateRAMBLER()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            RobotInputRAMBLER(rosLinear, -rosAngular);
        }

        private void SetParameters(ArticulationBody joint, float forceLimit)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            //drive.damping = damping;

            if (drive.targetVelocity < 0)
            {
                sentidoVelocidad = -1;
            }
            else if (drive.targetVelocity > 0)
            {
                sentidoVelocidad = 1;
            }
            joint.xDrive = drive;
        }

        private void SetSpeedRAMBLER(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }

            joint.xDrive = drive;
        }

        private void RobotInputRAMBLER(float speed, float rotSpeed) // m/s and rad/s
        {   
            // Check max values
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }

            float wR_Rotation = (speed / wheelRadius);
            float wL_Rotation = wR_Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);

            // Configure rotation
            if (rotSpeed != 0)
            {
                forceLimit = 800;
                //damping = 100;
                wR_Rotation = (wR_Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wL_Rotation = (wL_Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;

                SetParameters(wFL, 800);
                SetParameters(wFR, 800);
                SetParameters(wBL, 800);
                SetParameters(wBR, 800);
            }
            else if (sentidoVelocidad <= 0)
            {
                forceLimit = 400;
                //damping = 50;
                wR_Rotation *= Mathf.Rad2Deg;
                wL_Rotation *= Mathf.Rad2Deg;

                //CARACTERIZACION PARA ADELANTE
                SetParameters(wFL, 275);
                SetParameters(wFR, 275);
                SetParameters(wBL, 275);
                SetParameters(wBR, 275);
            }
            else
            {
                forceLimit = 400;
                //damping = 50;
                wR_Rotation *= Mathf.Rad2Deg;
                wL_Rotation *= Mathf.Rad2Deg;
                
                //CARACTERIZACION PARA ATRÁS
                SetParameters(wFL, 275);
                SetParameters(wFR, 275);
                SetParameters(wBL, 275);
                SetParameters(wBR, 275);
            }


            SetSpeedRAMBLER(wFL, wL_Rotation);
            SetSpeedRAMBLER(wFR, wR_Rotation);
            SetSpeedRAMBLER(wBL, wL_Rotation);
            SetSpeedRAMBLER(wBR, wR_Rotation);

            Debug.Log(speed);
            // Debug.Log(rotSpeed);
            // Debug.Log(wL_Rotation); //-174 130
            // Debug.Log(wR_Rotation); //174 130
            // Debug.Log("--------------\n");
        }
    }
}