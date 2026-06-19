/******************************************************
 * Autora: Alba Cuevas Fernandez
 * Grado en Ingenieria Electronica, Robotica y Mecatronica
 * Universidad de Malaga
 * Trabajo de Fin de Grado: Modelado y Simulacion del Robot Movil RAMBLER
 * Año: 2026
 ******************************************************/

using System;
using System.Runtime.CompilerServices;
using System.Text.RegularExpressions;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;

namespace RosSharp.Control
{
    public class RAMBLER_Suspension_Indv : MonoBehaviour
    {
        public string topicName = "/activate_suspension";

        public GameObject suspension_atras_izq;
        public GameObject suspension_atras_dcha;
        public GameObject suspension_delante_izq;
        public GameObject suspension_delante_dcha;
        public GameObject piston_atras_izq;
        public GameObject piston_atras_dcha;
        public GameObject piston_delante_izq;
        public GameObject piston_delante_dcha;

        private ArticulationBody susp_atras_izq;
        private ArticulationBody susp_atras_dcha;
        private ArticulationBody susp_del_izq;
        private ArticulationBody susp_del_dcha;
        private ArticulationBody p_den_atras_izq;
        private ArticulationBody p_den_atras_dcha;
        private ArticulationBody p_den_del_izq;
        private ArticulationBody p_den_del_dcha;

        private float forceLimit_piston_arriba = 150;
        private float forceLimit_piston_abajo = 1;

        private float forceLimit_susp_delante_arriba = 655;
        private float forceLimit_susp_atras_arriba = 640;

        private float forceLimit_susp_delante_abajo = 1;
        private float forceLimit_susp_atras_abajo = 1;

        public float ROSTimeout = 0.5f;
        private float lastActivateReceived = 0f;

        ROSConnection ros;
        private Int8Msg rosActivarSuspension;
        private string modo_suspension;

        void Start()
        {
            susp_atras_izq = suspension_atras_izq.GetComponent<ArticulationBody>();
            susp_atras_dcha = suspension_atras_dcha.GetComponent<ArticulationBody>();
            susp_del_izq = suspension_delante_izq.GetComponent<ArticulationBody>();
            susp_del_dcha = suspension_delante_dcha.GetComponent<ArticulationBody>();
            p_den_atras_izq = piston_atras_izq.GetComponent<ArticulationBody>();
            p_den_atras_dcha = piston_atras_dcha .GetComponent<ArticulationBody>();
            p_den_del_izq = piston_delante_izq.GetComponent<ArticulationBody>();
            p_den_del_dcha = piston_delante_dcha.GetComponent <ArticulationBody>();


            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<Int8Msg>("activate_suspension", ReceiveROSSusp);
        }

        // Recibe mensajes del topic activate_suspension
        void ReceiveROSSusp(Int8Msg aviso)
        {
            rosActivarSuspension = aviso;
            Debug.Log("Recibiendo: ");
            Debug.Log(rosActivarSuspension);

            RobotInput(rosActivarSuspension);

            lastActivateReceived = Time.time;
        }

        private void SetParameters(ArticulationBody joint, float target, float forceLimit)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.target = target;
            drive.forceLimit = forceLimit;

            // Actualización valores
            joint.xDrive = drive;
        }



        private void RobotInput(Int8Msg aviso)
        {
            switch (aviso.data)
            {
                case 0:
                    //CHASIS ABAJO COMPLETO
                    SetParameters(susp_del_izq, susp_del_izq.xDrive.lowerLimit, forceLimit_susp_delante_abajo);
                    SetParameters(susp_del_dcha, susp_del_dcha.xDrive.lowerLimit, forceLimit_susp_delante_abajo);

                    SetParameters(susp_atras_izq, susp_atras_izq.xDrive.upperLimit, forceLimit_susp_atras_abajo);
                    SetParameters(susp_atras_dcha, susp_atras_dcha.xDrive.upperLimit, forceLimit_susp_atras_abajo);

                    SetParameters(p_den_del_izq, p_den_del_izq.xDrive.lowerLimit, forceLimit_piston_abajo);
                    SetParameters(p_den_del_dcha, p_den_del_dcha.xDrive.lowerLimit, forceLimit_piston_abajo);

                    SetParameters(p_den_atras_izq, p_den_atras_izq.xDrive.upperLimit, forceLimit_piston_abajo);
                    SetParameters(p_den_atras_dcha, p_den_atras_dcha.xDrive.upperLimit, forceLimit_piston_abajo);

                    modo_suspension = "DESACTIVADAS";
                    break;
                case 2:
                    //TRASERA COMPLETO
                    SetParameters(susp_del_izq, susp_del_izq.xDrive.lowerLimit, forceLimit_susp_delante_abajo);
                    SetParameters(susp_del_dcha, susp_del_dcha.xDrive.lowerLimit, forceLimit_susp_delante_abajo);

                    SetParameters(susp_atras_izq, susp_atras_izq.xDrive.lowerLimit, forceLimit_susp_atras_arriba);
                    SetParameters(susp_atras_dcha, susp_atras_dcha.xDrive.lowerLimit, forceLimit_susp_atras_arriba);

                    SetParameters(p_den_del_izq, p_den_del_izq.xDrive.lowerLimit, forceLimit_piston_abajo);
                    SetParameters(p_den_del_dcha, p_den_del_dcha.xDrive.lowerLimit, forceLimit_piston_abajo);

                    SetParameters(p_den_atras_izq, p_den_atras_izq.xDrive.lowerLimit, forceLimit_piston_arriba);
                    SetParameters(p_den_atras_dcha, p_den_atras_dcha.xDrive.lowerLimit, forceLimit_piston_arriba);

                    modo_suspension = "ACTIVADAS (TRASERA)";
                    break;
                case 4:
                    //IZQ COMPLETO
                    SetParameters(susp_del_izq, susp_del_izq.xDrive.upperLimit, forceLimit_susp_delante_arriba);
                    SetParameters(susp_del_dcha, susp_del_dcha.xDrive.lowerLimit, forceLimit_susp_delante_abajo);

                    SetParameters(susp_atras_izq, susp_atras_izq.xDrive.lowerLimit, forceLimit_susp_atras_arriba);
                    SetParameters(susp_atras_dcha, susp_atras_dcha.xDrive.upperLimit, forceLimit_susp_atras_abajo);

                    SetParameters(p_den_del_izq, p_den_del_izq.xDrive.upperLimit, forceLimit_piston_arriba);
                    SetParameters(p_den_del_dcha, p_den_del_dcha.xDrive.lowerLimit, forceLimit_piston_abajo);

                    SetParameters(p_den_atras_izq, p_den_atras_izq.xDrive.lowerLimit, forceLimit_piston_arriba);
                    SetParameters(p_den_atras_dcha, p_den_atras_dcha.xDrive.upperLimit, forceLimit_piston_abajo);

                    modo_suspension = "ACTIVADAS (IZQUIERDA)";
                    break;
                case 5:
                    //CHASIS ARRIBA COMPLETO
                    SetParameters(susp_del_izq, susp_del_izq.xDrive.upperLimit, forceLimit_susp_delante_arriba);
                    SetParameters(susp_del_dcha, susp_del_dcha.xDrive.upperLimit, forceLimit_susp_delante_arriba);

                    SetParameters(susp_atras_izq, susp_atras_izq.xDrive.lowerLimit, forceLimit_susp_atras_arriba);
                    SetParameters(susp_atras_dcha, susp_atras_dcha.xDrive.lowerLimit, forceLimit_susp_atras_arriba);

                    SetParameters(p_den_del_izq, p_den_del_izq.xDrive.upperLimit, forceLimit_piston_arriba);
                    SetParameters(p_den_del_dcha, p_den_del_dcha.xDrive.upperLimit, forceLimit_piston_arriba);

                    SetParameters(p_den_atras_izq, p_den_atras_izq.xDrive.lowerLimit, forceLimit_piston_arriba);
                    SetParameters(p_den_atras_dcha, p_den_atras_dcha.xDrive.lowerLimit, forceLimit_piston_arriba);
                    
                    modo_suspension = "ACTIVADAS";
                    break;
                case 6:
                    //DCHA COMPLETO
                    SetParameters(susp_del_izq, susp_del_izq.xDrive.lowerLimit, forceLimit_susp_delante_abajo);
                    SetParameters(susp_del_dcha, susp_del_dcha.xDrive.upperLimit, forceLimit_susp_delante_arriba);

                    SetParameters(susp_atras_izq, susp_atras_izq.xDrive.upperLimit, forceLimit_susp_atras_abajo);
                    SetParameters(susp_atras_dcha, susp_atras_dcha.xDrive.lowerLimit, forceLimit_susp_atras_arriba);

                    SetParameters(p_den_del_izq, p_den_del_izq.xDrive.lowerLimit, forceLimit_piston_abajo);
                    SetParameters(p_den_del_dcha, p_den_del_dcha.xDrive.upperLimit, forceLimit_piston_arriba);

                    SetParameters(p_den_atras_izq, p_den_atras_izq.xDrive.upperLimit, forceLimit_piston_abajo);
                    SetParameters(p_den_atras_dcha, p_den_atras_dcha.xDrive.lowerLimit, forceLimit_piston_arriba);

                    modo_suspension = "ACTIVADAS (DERECHA)";
                    break;
                case 8:
                    //FRONTAL COMPLETO
                    SetParameters(susp_del_izq, susp_del_izq.xDrive.upperLimit, forceLimit_susp_delante_arriba);
                    SetParameters(susp_del_dcha, susp_del_dcha.xDrive.upperLimit, forceLimit_susp_delante_arriba);

                    SetParameters(susp_atras_izq, susp_atras_izq.xDrive.upperLimit, forceLimit_susp_atras_abajo);
                    SetParameters(susp_atras_dcha, susp_atras_dcha.xDrive.upperLimit, forceLimit_susp_atras_abajo);

                    SetParameters(p_den_del_izq, p_den_del_izq.xDrive.upperLimit, forceLimit_piston_arriba);
                    SetParameters(p_den_del_dcha, p_den_del_dcha.xDrive.upperLimit, forceLimit_piston_arriba);

                    SetParameters(p_den_atras_izq, p_den_atras_izq.xDrive.upperLimit, forceLimit_piston_abajo);
                    SetParameters(p_den_atras_dcha, p_den_atras_dcha.xDrive.upperLimit, forceLimit_piston_abajo);

                    modo_suspension = "ACTIVADAS (FRONTAL)";
                    break;
            }

            Debug.Log("Modo de suspensiones: " + modo_suspension);
            Debug.Log("--------------\n");

        }
    }
}