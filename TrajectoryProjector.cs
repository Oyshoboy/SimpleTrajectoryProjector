using UnityEngine;
using System.Collections.Generic;

namespace SimulationUI
{
    public enum TrajectoryDirections
    {
        RollingOut,
        RollingIn
    }
    
    public enum TrajectoryState
    {
        In,
        Magnitude,
        Out,
    }
    
    public class TrajectoryProjector : MonoBehaviour
    {
        #region variables
        [Header("References")]
        public GameObject pointObject;
        public LineRenderer lineRenderer;
        public Rigidbody velocitySampler;
        public Rigidbody relativeSampler;
        
        [Header("Direction alignment")]
        [Tooltip("For example head, in order to align aim with sampler direction")]
        public Transform directionSampler;
        public bool isOnAlignedDirection;
        public float alignedDirectionThreshold = 0.5f;
        
        [Header("Configuration")]
        public float radius = 0.033f;
        public LayerMask layerMask = ~0;
        public float magnitudeThreshold = 0.2f;
        public int handsMagnitudeBufferSize = 20;

        [Header("Physics")]
        public float launchMagnitude = 10f;
        public bool useGravity = true;
        public float stepSize = 0.05f;
        public int resolutionIterations = 60;
        public float minimumDistance = 4f;

        [Header("Smoothing")]
        [Range(1, 100)]
        public int smoothingFactor = 5;
        [Tooltip("Blend curve between instant and smoothed trajectories")]
        public AnimationCurve blendCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);

        // [Header("Debug")]
        [HideInInspector]public TrajectoryDirections trajectoryDirection;
        [HideInInspector]public TrajectoryState trajectoryState;
        [HideInInspector]public float totalDistance;
        [HideInInspector]public int currentResolution;

        [HideInInspector]public List<Vector3> tempPointsBuffer;
        [HideInInspector]public Vector3[] instantPointsBuffer;
        [HideInInspector]public Vector3[] smoothPointsBuffer;
        [HideInInspector]public List<float> handsMagnitudeBuffer;
        [HideInInspector]public List<Vector3> pointsBuffer;
        [HideInInspector]public List<Vector3> initialVelocityBuffer;
        [HideInInspector]public List<float> distanceBuffer;
        #endregion

        #region built-in methods
        void Update()
        {
            TrajectoryGlobalHandler();
        }
        #endregion

        #region runtime operations
        private void SamplerMagnitudeHandler()
        {
            if (!velocitySampler) return;
            var samplerMag = velocitySampler.velocity.magnitude;
            var res = relativeSampler ? Mathf.Abs(relativeSampler.velocity.magnitude - samplerMag) : samplerMag;
            StoreDataToBuffer(handsMagnitudeBuffer, res, handsMagnitudeBufferSize);
        }
        #endregion

        #region velues getters
        private float AverageDistance()
        {
            if (IsNullOrEmpty(distanceBuffer)) return 0;

            return AverageFromBuffer(distanceBuffer);
        }

        private Vector3 InitialPointPosition()
        {
            return transform.position;
        }

        private Vector3 SmoothInitialVector()
        {
            return AverageFromBuffer(initialVelocityBuffer);
        }
        #endregion

        #region togglers
        private void ToggleTrajectoryDirection(TrajectoryDirections stateOrdered)
        {
            if (Equals(trajectoryDirection, stateOrdered)) return;
            trajectoryDirection = stateOrdered;
        }

        public virtual void ToggleTrajectoryState(TrajectoryState stateOrdered)
        {
            if (Equals(trajectoryState, stateOrdered)) return;
            if (stateOrdered == TrajectoryState.Magnitude && trajectoryState == TrajectoryState.In) return; 
            trajectoryState = stateOrdered;
        }
        private void DisableTrajectory()
        {
            ClearBuffers();
            lineRenderer.positionCount = 0;
            currentResolution = 0;
            TogglePointers(false);
        }
        
        private void TogglePointers(bool state)
        {
            if(pointObject.activeSelf != state)
            {
                pointObject.SetActive(state);
            }
        }
        #endregion

        #region trajectory calculations

        private void TrajectoryGlobalHandler()
        {

            SamplerMagnitudeHandler();

            if (launchMagnitude <= 0)
            {
                DisableTrajectory();
                return;
            }

            if (MagnitudeMoreThanThreshold())
            {
                ToggleTrajectoryState(TrajectoryState.Magnitude);
                ToggleTrajectoryDirection(TrajectoryDirections.RollingIn);
            }
            else
            {
                if (!pointObject.activeSelf)
                {
                    TogglePointers(true);
                    ToggleTrajectoryDirection(TrajectoryDirections.RollingOut);
                }
            }

            CalculateInstantTrajectory();
            DisplaySmoothPoint();
        }
        
        
        private Vector3 RunTrajectorySimulation(Vector3 inVel, Vector3 gravity, float step, Vector3 initialPoint)
        {
            totalDistance = 0;
            var totalIterations = 0;
            // first 3 axes are point coords, 4th is for total iterations
            var result = Vector3.zero;
            tempPointsBuffer.Clear();
            tempPointsBuffer.Add(initialPoint);
            
            for (var i = 0; i < resolutionIterations; i++)
            {
                var time = i * step;
                var displacement = inVel * time + gravity * (0.5f * time * time);
                tempPointsBuffer.Add(initialPoint + displacement);

                var previousPoint = tempPointsBuffer[tempPointsBuffer.Count - 2];
                var currentPoint = tempPointsBuffer[tempPointsBuffer.Count - 1];
                // get distance from prevPoint to nextPoint
                var distance = Vector3.Distance(previousPoint, currentPoint);
                var checkedHitPoint = SphereCastDetected(previousPoint, currentPoint, distance);

                totalIterations++;
                if (checkedHitPoint != Vector3.zero)
                {
                    totalDistance += Vector3.Distance(previousPoint, checkedHitPoint);
                    result = checkedHitPoint;
                    break;
                }
                else
                {
                    totalDistance += distance;
                    
                    if (totalIterations == resolutionIterations)
                    {
                        // last frame achieved
                        var lastPoint = currentPoint;
                        result = lastPoint;
                        break;
                    }
                }
            }

            return result;
        }


        private void CalculateInstantTrajectory()
        {
            var initialVelocity = transform.forward * launchMagnitude;
            var gravity = useGravity ? Physics.gravity : Vector3.zero;
            var initialPoint = InitialPointPosition();

            var totalIterations = 0;
            var simPoint = RunTrajectorySimulation(initialVelocity, gravity, stepSize, initialPoint);

            if (simPoint != Vector3.zero)
            {
                TogglePointers(true);
                StoreDataToBuffer(distanceBuffer, totalDistance, smoothingFactor);
                StoreDataToBuffer(pointsBuffer, simPoint, smoothingFactor);
                StoreDataToBuffer(initialVelocityBuffer, initialVelocity, smoothingFactor);
            }

            var firstPassLocalPoints = new List<Vector3>();

            if (totalDistance < minimumDistance || trajectoryDirection == TrajectoryDirections.RollingIn)
            {
                TogglePointers(false);
            }

            if (AverageDistance() < minimumDistance || trajectoryDirection == TrajectoryDirections.RollingIn)
            {
                if(currentResolution > totalIterations) currentResolution = totalIterations;

                if (currentResolution > currentResolution / 3)
                {
                    currentResolution -= 4;
                }
                else
                {
                    currentResolution -= 1;
                }

                if (currentResolution <= 1)
                {
                    ToggleTrajectoryState(TrajectoryState.In);
                    DisableTrajectory();
                    return;
                }
            }
            else
            {
                if (resolutionIterations > currentResolution)
                {
                    if (currentResolution == 0)
                    {
                        ToggleTrajectoryState(TrajectoryState.Out);
                    }
                    currentResolution += 2;
                }
            }
            
            for (int i = 0; i < currentResolution; i++)
            {
                if(i > tempPointsBuffer.Count - 1)
                {
                    break;
                }
                firstPassLocalPoints.Add(tempPointsBuffer[i]);
            }

            instantPointsBuffer = firstPassLocalPoints.ToArray();
        }
        
        private void CalculateSmoothTrajectory()
        {
            var initialVelocity = SmoothInitialVector();
            var gravity = useGravity ? Physics.gravity : Vector3.zero;
            var initialPoint = InitialPointPosition();

            // second pass trajectory calculations, based on cached data
            RunTrajectorySimulation(initialVelocity, gravity, stepSize, initialPoint);
            smoothPointsBuffer = tempPointsBuffer.ToArray();
            CalculateBlendCurve();
        }

        private void CalculateBlendCurve()
        {
            var newHybridPoints = new List<Vector3>();
            var maxValue = instantPointsBuffer.Length;

            for (int i = 0; i < maxValue; i++)
            {
                // blend between instant and smooth
                var blend = i / (float)maxValue;
                blend = blendCurve.Evaluate(blend);
                if(instantPointsBuffer.Length - 1 < i || smoothPointsBuffer.Length -1 < i) break;
                var instantPoint = instantPointsBuffer[i];
                var smoothPoint = smoothPointsBuffer[i];
                var hybridPoint = Vector3.Lerp(instantPoint, smoothPoint, blend);
                newHybridPoints.Add(hybridPoint);
            }

            lineRenderer.positionCount = newHybridPoints.Count;
            lineRenderer.SetPositions(newHybridPoints.ToArray());
        }
        #endregion
        
        #region generic helpers
        bool IsNullOrEmpty<T>(IList<T> list) => list == null || list.Count == 0;
        
        private Vector3 SphereCastDetected(Vector3 prevPoint, Vector3 nextPoint, float distance)
        {
            Vector3 direction = nextPoint - prevPoint;
            RaycastHit hit;

            if (Physics.SphereCast(prevPoint, radius, direction, out hit, distance, layerMask))
            {
                return hit.point;
            }
            else
            {
                return Vector3.zero;
            }
        }
        
        private bool MagnitudeMoreThanThreshold()
        {
            if (!velocitySampler) return false;
            
            if (!AllignedWithSamplerDirection())
            {
                return true;
            }
            
            return AverageFromBuffer(handsMagnitudeBuffer) > magnitudeThreshold;
        }

        private bool AllignedWithSamplerDirection()
        {
            if (!directionSampler) return true;
            
            var handDirection = velocitySampler.transform.forward;
            var headDirection = directionSampler.transform.forward;
            
            // check if both objects are facing the same direction
            if (Vector3.Dot(handDirection, headDirection) > alignedDirectionThreshold)
            {
                isOnAlignedDirection = true;
                return true;
            }
            else
            {
                isOnAlignedDirection = false;
                return false;
            }
        }

        private void MovePointerPointToTheHitPoint(Vector3 pos)
        {
            pointObject.transform.position = pos;
        }

        private void DisplaySmoothPoint()
        {
            if (IsNullOrEmpty(pointsBuffer)) return;
            var averagePoint = AverageFromBuffer(pointsBuffer);
            
            CalculateSmoothTrajectory();
            MovePointerPointToTheHitPoint(averagePoint);
        }
        #endregion
        
        #region buffer operations
        private void StoreDataToBuffer<T>(IList<T> buffer, T data, int bufferSize)
        {
            // check if buffer is full 
            if (buffer.Count > bufferSize)
            {
                buffer.RemoveAt(0);
            }
            
            buffer.Add(data);
            
            // second check if buffer is full, in order to reduce the buffer size on runtime
            if (buffer.Count > bufferSize)
            {
                buffer.RemoveAt(0);
            }
        }
        
        private void ClearBuffers()
        {
            pointsBuffer.Clear();
            distanceBuffer.Clear();
            initialVelocityBuffer.Clear();
            smoothPointsBuffer = new Vector3[]{};
            instantPointsBuffer = new Vector3[]{};
        }
        
        // generic method for getting the average of a buffer
        private float AverageFromBuffer(List<float> buffer)
        {
            var sum = 0f;
            if (IsNullOrEmpty(buffer)) return sum;
            foreach (var value in buffer)
            {
                sum += value;
            }
            
            return sum / buffer.Count;
        }
        
        private Vector3 AverageFromBuffer(List<Vector3> buffer)
        {
            var sum = Vector3.zero;
            if (IsNullOrEmpty(buffer)) return sum;
            foreach (var value in buffer)
            {
                sum += value;
            }
            return sum / buffer.Count;
        }
        #endregion
    }
}
