using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

using AGXUnity.Utils;
using AGXUnity;
namespace ML.Scripts
{
    public class WheelLoaderAgent : Agent
    {
        public GameObject WheelLoaderGameObject { get; private set; } = null;

        public AGXUnity.Model.WheelLoader WheelLoader { get; private set; } = null;

        public AGXUnity.Model.WheelLoaderInputController WheelLoaderInput { get; private set; } = null;

        public Transform ReferenceTransform { get; private set; } = null;

        public bool RandomWayPoints { get; private set; } = true;

        public GameObject WheelLoaderResource
        {
            get
            {
                if (s_wheelLoaderResource == null)
                {
                    s_wheelLoaderResource = Resources.Load<GameObject>("wheel_loader_DL300");
                }
                return s_wheelLoaderResource;
            }
        }

        public GameObject WayPoint
        {
            get
            {
                return m_wayPoints[m_wayPointsIndex];
            }
        }

        public override void OnEpisodeBegin()
        {
            if (m_wayPoints == null)
            {
                GetOrCreateTargetObjects();
                CreateLineThroughTargets();
            }
            if (m_failedEpisode)
            {
                m_wayPointsIndex = -1;
                m_failedEpisode = false;
            }
            // Reset vehicle and target transforms
            if (m_wayPointsIndex >= m_wayPoints.Length - 1 || m_wayPointsIndex < 0)
            {
                ResetEnvironment();
            }

            ReferenceTransform = WheelLoader.FrontBodyObserver.transform;
            InitialReferencePosition = ReferencePosition;
            m_wayPointsIndex += 1;
        }


        private void ResetEnvironment()
        {
            if (WheelLoaderGameObject != null)
                DestroyImmediate(WheelLoaderGameObject);
            // first time starting
            else
            {
                // Turn off automatic environment stepping
                Academy.Instance.AutomaticSteppingEnabled = false;
                // Make sure environment steps in simulation post.
                Simulation.Instance.StepCallbacks.PostStepForward += Academy.Instance.EnvironmentStep;
            }

            if (Terrain.activeTerrain != null)
                Terrain.activeTerrain.GetComponent<AGXUnity.Model.DeformableTerrain>().ResetHeights();
            Simulation.Instance.Native.garbageCollect();

            IsDisabled = false;
            // Load wheel loader and components on it.
            WheelLoaderGameObject = Instantiate(WheelLoaderResource);
            WheelLoaderGameObject.transform.position = new Vector3(30.0f, 0.07f, 25f);
            WheelLoaderGameObject.transform.rotation = Quaternion.Euler(-90, 90, 0);
            WheelLoader = WheelLoaderGameObject.AddComponent<AGXUnity.Model.WheelLoader>().GetInitialized<AGXUnity.Model.WheelLoader>();
            ShovelComponent();

            foreach (var script in WheelLoaderGameObject.GetComponentsInChildren<ScriptComponent>())
                script.GetInitialized<ScriptComponent>();

            WheelLoader.Engine.setDischargeCoefficient(0.5);
            WheelLoader.TireProperties.LateralStiffness = 1.0E12f;
            WheelLoader.TireProperties.RadialStiffness = 1.0E12f;

            ReferenceTransform = WheelLoader.FrontBodyObserver.transform;
            InitialReferencePosition = ReferencePosition;

            // Position every target position
            m_wayPointsIndex = 0;
            if (RandomWayPoints)
                RandomizeTargetPositions();

            DrawLinesThroughTargets();
        }

        private void ShovelComponent()
        {
            bucket = WheelLoader.transform.Find("Bucket").gameObject;

            Debug.Assert(bucket.GetComponent<AGXUnity.RigidBody>() != null);
            shovel = bucket.GetComponent<AGXUnity.Model.DeformableTerrainShovel>() == null ?
            bucket.AddComponent<AGXUnity.Model.DeformableTerrainShovel>() :
            bucket.GetComponent<AGXUnity.Model.DeformableTerrainShovel>();

            ShovelMeshConfiguration();

            Terrain.activeTerrain.GetComponent<AGXUnity.Model.DeformableTerrain>().Add(shovel);
        }

        private void ShovelMeshConfiguration()
        {
            GameObject bucket_tiremesh = bucket.transform.Find("Bucket_Trimesh").gameObject;
            GameObject bucket4_tiremesh = bucket.transform.Find("Bucket4_Trimesh").gameObject;

            shovel.TopEdge.Start.SetParent(bucket_tiremesh);
            shovel.TopEdge.Start.LocalPosition = new Vector3(1.219116f, -6.782373f, -2.222016f);
            shovel.TopEdge.Start.LocalRotation = Quaternion.Euler(62.33485f, 209.7749f, 302.5015f);
            shovel.TopEdge.End.SetParent(bucket_tiremesh);
            shovel.TopEdge.End.LocalPosition = new Vector3(1.906602f, -4.112466f, -1.007924f);
            shovel.TopEdge.End.LocalRotation = Quaternion.Euler(297.6652f, 29.77492f, 237.4985f);

            shovel.CuttingEdge.Start.SetParent(bucket_tiremesh);
            shovel.CuttingEdge.Start.LocalPosition = new Vector3(0.2560971f, -6.270654f, -2.83864f);
            shovel.CuttingEdge.Start.LocalRotation = Quaternion.Euler(63.93867f, 207.5661f, 329.1487f);
            shovel.CuttingEdge.End.SetParent(bucket_tiremesh);
            shovel.CuttingEdge.End.LocalPosition = new Vector3(0.8828747f, -3.53994f, -1.648815f);
            shovel.CuttingEdge.End.LocalRotation = Quaternion.Euler(296.0613f, 27.56611f, 210.8513f);

            shovel.CuttingDirection.Start.SetParent(bucket4_tiremesh);
            shovel.CuttingDirection.Start.LocalPosition = new Vector3(0.5f, -5f, -2.3f);
            shovel.CuttingDirection.Start.LocalRotation = Quaternion.Euler(356.5f, 123f, 40.7f);

        }

        public override void CollectObservations(VectorSensor sensor)
        {
            if (IsDisabled)
            {
                int sensorLimitation = 8;

                for (int i = 0; i < sensorLimitation; i++)
                {
                    sensor.AddObservation(0.0f);
                }
                return;
            }

            // Agent will strive to put this to zero
            sensor.AddObservation(DistanceToTarget);
            // Agent must try to point towards target
            sensor.AddObservation(LocalDirectionToTarget.x);
            sensor.AddObservation(LocalDirectionToTarget.z);
            // On the way upwards or downwards
            sensor.AddObservation(FrontBodyAngle);
            // Agent must try align towards the next target
            sensor.AddObservation(AngleBetweenTargetForward);
            // Current speed in the current direction
            sensor.AddObservation(WheelLoader.Speed);

            // Wheel loader actuator observations
            // sensor.AddObservation((float)WheelLoader.SteeringHinge.Native.asHinge().getAngle());
            // sensor.AddObservation((float)WheelLoader.SteeringHinge.Native.asHinge().getCurrentSpeed());
            sensor.AddObservation((float)WheelLoader.Engine.getRPM());

            // wheel loader bucket current total weight
            sensor.AddObservation((float)Terrain.activeTerrain.GetComponent<AGXUnity.Model.DeformableTerrain>().Native.getDynamicMass(shovel.Native));
        }

        public override void OnActionReceived(float[] vectorAction)
        {
            if (IsDisabled)
                return;

            if (m_failedEpisode = CheckFailedEpisode())
            {
                DoEndEpisode(0.0f);
                return;
            }

            float steer = 0.1f * Math.Clamp(vectorAction[1], -1.0f, 1.0f);
            float throttle = Math.Clamp(vectorAction[0], 0.0f, 1.0f);
            float elevate = Math.Clamp(vectorAction[2], -1.0f, 1.0f);
            float tilt = Math.Clamp(vectorAction[3], -1.0f, 1.0f);


            // Update the control signals on the actuators. Important to do this before stepping!
            SetControlSignals(steer, throttle, 0.0f);
            SetControlSignalsShovel(elevate, tilt);
            float reward = -0.0001f;
            bool end = false;
            if (CheckFinish())
            {
                var r_pos = GetPositionReward();
                var r_rot = GetRotationReward();
                var r_pose = r_pos * r_rot;
                reward = r_pose;
                end = true;
            }

            SetReward(reward);
            if (end)
            {
                EndEpisode();
            }
        }



        public override void Heuristic(float[] actionsOut)
        {
            if (IsDisabled)
                return;

            if (WheelLoaderGameObject.GetComponent<AGXUnity.Model.WheelLoaderInputController>() == null)
            {
                WheelLoaderInput = WheelLoaderGameObject.AddComponent<AGXUnity.Model.WheelLoaderInputController>();
#if ENABLE_INPUT_SYSTEM
                if (WheelLoaderInput.InputAsset == null)
                    WheelLoaderInput.InputAsset = Resources.Load<UnityEngine.InputSystem.InputActionAsset>("Input/AGXUnityInputControls");
#endif
                WheelLoaderInput.InputMode = AGXUnity.Model.WheelLoaderInputController.ActionMode.Devices;
                WheelLoaderInput.GetInitialized<ScriptComponent>();
            }

            actionsOut[0] = WheelLoaderInput.Throttle;
            // actionsOut[1] = -1.0f * WheelLoaderInput.Steer;
        }

        protected override void OnDisable()
        {
            IsDisabled = true;
            base.OnDisable();
        }

        private float GetPositionReward()
        {
            float reward = 0.0f;
            if (DistanceToTarget < 0.5f)
                reward = 1.0f;
            else
                reward = Mathf.Exp(-Mathf.Pow(DistanceToTarget, 2) / 2.0f);
            return reward;
        }

        private float GetRotationReward()
        {
            return Mathf.Max(0.0f, 5 * Vector3.Dot(ForwardDirection, WayPoint.transform.forward) - 4.0f);
        }

        private bool CheckFailedEpisode()
        {
            bool end = false;
            var failOutOfBounds = DistanceToTarget > InitialDistanceToTarget + 2.0f;
            if (failOutOfBounds || StepCount >= MaxStep)
            {
                end = true;
            }
            return end;
        }

        private bool CheckFinish()
        {
            // TargetObject transform z direction is the vector to pass
            var A = TargetPosition;
            var B = TargetPosition + WayPoint.transform.right.normalized;
            var P = ReferencePosition;

            var d = (P.x - A.x) * (B.z - A.z) - (P.z - A.z) * (B.x - A.x);
            if (d < 0)
            {
                return true;
            }
            return false;
        }

        private void DoEndEpisode(float reward)
        {
            SetReward(reward);
            EndEpisode();
        }

        private float InitialDistanceToTarget
        {
            get
            {
                return Vector3.Distance(InPlane(InitialReferencePosition), InPlane(TargetPosition));
            }
        }

        private Vector3 TargetPosition
        {
            get
            {
                return WayPoint.transform.position;
            }
        }

        private Vector3 InitialReferencePosition { get; set; }

        private float DistanceToTarget
        {
            get
            {
                return Vector3.Distance(InPlane(TargetPosition), InPlane(ReferencePosition));
            }
        }

        private Vector3 DirectionToTarget
        {
            get
            {
                return Vector3.Normalize(InPlane(TargetPosition) - InPlane(ReferencePosition));
            }
        }

        private Vector3 ForwardDirection
        {
            get
            {
                return InPlane(ReferenceTransform.forward).normalized;
            }
        }

        private float AngleBetweenTargetForward
        {
            get
            {
                float angle = Mathf.Acos(Mathf.Clamp(Vector3.Dot(ForwardDirection, WayPoint.transform.forward), -1.0f, 1.0f));
                Vector3 cross = Vector3.Cross(ForwardDirection, WayPoint.transform.forward);
                if (Vector3.Dot(Vector3.up, cross) < 0)
                    angle = -angle;

                return angle;
            }
        }

        private Vector3 LocalDirectionToTarget
        {
            get
            {
                return ReferenceTransform.InverseTransformVector(DirectionToTarget);
            }
        }

        private float FrontBodyAngle
        {
            get
            {
                return ReferenceTransform.forward.y;
            }
        }

        private Vector3 ReferencePosition
        {
            get
            {
                return InPlane(ReferenceTransform.position);
            }
        }

        private bool IsDisabled { get; set; } = true;

        private Vector3 InPlane(Vector3 v)
        {
            v.y = 0.0f;
            return v;
        }

        private void SetSpeed(Constraint constraint, float speed)
        {
            var motorEnable = !Math.EqualsZero(speed);
            var mc = constraint.GetController<TargetSpeedController>();
            var lc = constraint.GetController<LockController>();
            mc.Enable = motorEnable;
            mc.Speed = speed;
            if (!motorEnable && !lc.Enable)
                lc.Position = constraint.GetCurrentAngle();
            lc.Enable = !motorEnable;
        }

        private void SetBrake(float value)
        {
            var brakeTorque = value * 1.5E5f;
            WheelLoader.BrakeHinge.getMotor1D().setEnable(value > 0.0f);
            WheelLoader.BrakeHinge.getMotor1D().setSpeed(0.0f);
            WheelLoader.BrakeHinge.getMotor1D().setForceRange(-brakeTorque, brakeTorque);
        }

        private void SetThrottle(float value)
        {
            WheelLoader.Engine.setThrottle(value);
        }

        private void SetTilt(float value)
        {
            SetSpeed(WheelLoader.TiltPrismatic, 0.25f * value);
        }

        private void SetElevate(float value)
        {
            var speed = 0.3f * value;
            foreach (var prismatic in WheelLoader.ElevatePrismatics)
                SetSpeed(prismatic, speed);
        }

        private void SetControlSignals(float steer, float throttle, float brake)
        {
            // SetSpeed(WheelLoader.SteeringHinge, steer);

            var speed = WheelLoader.Speed;
            var idleSpeed = 0.05f;

            if (Math.EqualsZero(throttle) && Math.EqualsZero(brake))
            {
                SetThrottle(0.0f);
                if (Mathf.Abs(speed) > idleSpeed)
                    SetBrake(0.1f);
                else
                    SetBrake(1.0f);
            }
            else
            {
                if (throttle > 0.0f)
                {
                    // Throttle down but going backwards. Brake.
                    if (speed < -idleSpeed)
                    {
                        SetThrottle(0.0f);
                        SetBrake(throttle);
                    }
                    else
                    {
                        WheelLoader.GearBox.setGear(1);
                        SetThrottle(throttle);
                        SetBrake(0.0f);
                    }
                }
                else if (brake > 0.0f)
                {
                    // Brake down and going forward. Brake.
                    if (speed > idleSpeed)
                    {
                        SetThrottle(0.0f);
                        SetBrake(brake);
                    }
                    else
                    {
                        WheelLoader.GearBox.setGear(0);
                        SetThrottle(brake);
                        SetBrake(0.0f);
                    }
                }
            }
        }

        private void SetControlSignalsShovel(float elevate, float tilt)
        {
            SetTilt(tilt);
            SetElevate(elevate);
        }

        private void RandomizeTargetPositions()
        {
            WayPoint.transform.SetPositionAndRotation(ReferenceTransform.position, ReferenceTransform.rotation);
            WayPoint.transform.Rotate(0, Random.Range(-5.0f, 5.0f), 0);
            for (int i = 1; i < m_wayPoints.Length; i++)
            {
                SampleRandomTargetPos(m_wayPoints[i], m_wayPoints[i - 1].transform, SampleWayPointDistance());
            }
        }

        private void DrawLinesThroughTargets()
        {
            m_line.positionCount = m_wayPoints.Length;
            for (int i = 0; i < m_wayPoints.Length; i++)
            {
                m_line.SetPosition(i, m_wayPoints[i].transform.position);
            }
        }

        private float SampleWayPointDistance()
        {
            return Academy.Instance.EnvironmentParameters.GetWithDefault("wheel_loader_curriculum", 11.0f);
        }

        private void SampleRandomTargetPos(GameObject target, Transform start, float distance)
        {
            target.transform.rotation = start.rotation;

            var dir = InPlane(start.forward).normalized;
            target.transform.position = InPlane(start.position) + distance * dir;
            target.transform.position = target.transform.position + 2.0f * Vector3.up;

            // Rotate the target to find an angle the vehicle should be at when aiming at target
            float angle = Random.Range(-3 * distance, 3 * distance);
            target.transform.Rotate(0, angle, 0);
        }

        private void CreateLineThroughTargets()
        {
            m_lineGO = new GameObject("Line");
            m_lineGO.AddComponent<LineRenderer>();
            m_line = m_lineGO.GetComponent<LineRenderer>();
            m_line.startWidth = 0.1f;
            m_line.endWidth = 0.1f;
            m_line.material = new Material(Shader.Find("Sprites/Default"));
            m_line.startColor = Color.grey;
            m_line.endColor = Color.grey;
        }

        private void GetOrCreateTargetObjects()
        {
            var to = GameObject.Find("WayPoints");
            if (to != null)
            {
                m_wayPoints = new GameObject[to.transform.childCount];
                for (int i = 0; i < to.transform.childCount; i++)
                {
                    m_wayPoints[i] = to.transform.GetChild(i).gameObject;
                }
                RandomWayPoints = false;
                return;
            }

            // create a number of sequential targets game objects
            m_wayPoints = new GameObject[6];
            var mat = new Material(Shader.Find("Sprites/Default"));
            for (int i = 0; i < m_wayPoints.Length; i++)
            {
                m_wayPoints[i] = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                m_wayPoints[i].GetComponent<MeshRenderer>().material = mat;
                mat.color = Color.black;
            }
        }

        private AGXUnity.Rendering.GizmosData m_gizmos = new AGXUnity.Rendering.GizmosData() { Offset = Vector3.up };
        private static GameObject s_wheelLoaderResource = null;
        private GameObject[] m_wayPoints = null;
        private GameObject m_lineGO = null;
        private LineRenderer m_line = null;
        private GameObject bucket = null;
        private AGXUnity.Model.DeformableTerrainShovel shovel = null;
        private int m_wayPointsIndex = -1;
        private bool m_failedEpisode = false;
    }
}