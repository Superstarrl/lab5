using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace AvoiderPlugin
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(NavMeshAgent))]
    public class Avoider : MonoBehaviour
    {
        [SerializeField] private Transform avoidee;

        public float dangerRadius = 8f;
        public float searchRadius = 15f;
        public float escapeSpeed = 3.5f;
        public float replanCooldown = 0.35f;

        public float poissonMinDistance = 2.0f;
        public int poissonAttemptsPerPoint = 25;

        public bool useFOV = true;
        public float fovDegrees = 120f;
        public LayerMask obstacleMask = Physics.DefaultRaycastLayers;
        public float eyeHeight = 1.6f;

        public bool showGizmos = true;
        public bool showSampleRays = false;

        // anti jitter fix (?)
        public float minMoveDistance = 1.0f;         // ignore destinations closer
        public float stickTargetUntilWithin = 0.7f;  // keep current target until agent gets this close
        public float replanIfAvoideeMovesBy = 1.5f;  // only replan early if avoidee moved at least this far

        // gizmo spokes
        public bool showRadialSpokes = true;
        public int spokes = 48;          // density of rays
        public float spokesRadius = 12f; // default length (you can also set = searchRadius)
        public bool stopSpokeAtHit = true;
        public bool showLocalAxes = true;
        public float axisLen = 2f;

        // colors
        public Color spokeClear = new Color(1f, 0f, 0f, 0.9f); // red (no hit)
        public Color spokeHit = new Color(0.9f, 0.2f, 0.2f, 1f);
        public Color axisXColor = Color.red;
        public Color axisZColor = Color.green;

        NavMeshAgent agent;
        readonly List<Vector3> lastSamples = new();
        readonly List<bool> lastHiddenFlags = new();
        Vector3? lastChosen;
        Vector3 lastAvoideePlanPos;

        void Reset()
        {
            var maybePlayer = GameObject.FindGameObjectWithTag("Player");
            if (maybePlayer) avoidee = maybePlayer.transform;
        }

        void OnValidate()
        {
            if (!agent) agent = GetComponent<NavMeshAgent>();
            if (agent && escapeSpeed < 0.1f) escapeSpeed = Mathf.Max(0.1f, agent.speed);
        }

        void Awake()
        {
            agent = GetComponent<NavMeshAgent>();
            if (!agent)
            {
                Debug.LogError("[Avoider] NavMeshAgent required.");
                enabled = false;
                return;
            }

            agent.autoBraking = true;
            agent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;
        }

        IEnumerator Start()
        {
            if (!NavMesh.SamplePosition(transform.position, out _, 2f, NavMesh.AllAreas))
                Debug.LogWarning("[Avoider] No NavMesh found near this object. Bake a NavMesh.");

            if (!avoidee)
                Debug.LogWarning("[Avoider] Avoidee is not assigned. Drag your Player here.");

            while (true)
            {
                if (!avoidee)
                {
                    yield return null;
                    continue;
                }

                FaceTargetFlat(avoidee.position);

                float dist = Vector3.Distance(transform.position, AvoideePos());
                if (dist > dangerRadius)
                {
                    if (!agent.pathPending && agent.remainingDistance < 0.25f)
                        agent.isStopped = true;

                    lastChosen = null;
                    yield return null;
                    continue;
                }

                // threatened, choose to replan or continue
                agent.isStopped = false;
                agent.speed = escapeSpeed;

                bool needNewPlan = true;

                if (lastChosen.HasValue)
                {
                    // keep current target unless we're close or it stopped being hidden
                    float distToTarget = Vector3.Distance(transform.position, lastChosen.Value);
                    if (distToTarget > stickTargetUntilWithin && IsHiddenFromAvoidee(lastChosen.Value))
                    {
                        // only replan if the avoidee has moved significantly since agent chose the target
                        float avoideeDelta = Vector3.Distance(AvoideePos(), lastAvoideePlanPos);
                        if (avoideeDelta < replanIfAvoideeMovesBy)
                            needNewPlan = false;
                    }
                }

                if (needNewPlan)
                {
                    if (FindHiddenEscape(out var best))
                    {
                        if ((best - transform.position).sqrMagnitude >= (minMoveDistance * minMoveDistance))
                        {
                            lastChosen = best;
                            lastAvoideePlanPos = AvoideePos();
                            agent.SetDestination(best);
                        }
                    }
                    else
                    {
                        // flee opposite direction and snap to NavMesh (fallback)
                        var flee = transform.position + (transform.position - AvoideePos()).normalized * Mathf.Max(4f, dangerRadius * 0.75f);
                        if (NavMesh.SamplePosition(flee, out var hit, 4f, NavMesh.AllAreas))
                        {
                            if ((hit.position - transform.position).sqrMagnitude >= (minMoveDistance * minMoveDistance))
                            {
                                lastChosen = hit.position;
                                lastAvoideePlanPos = AvoideePos();
                                agent.SetDestination(hit.position);
                            }
                        }
                        else
                        {
                            lastChosen = null;
                        }
                    }
                }

                yield return new WaitForSeconds(replanCooldown);
            }
        }

        Vector3 AvoideePos()
        {
            var p = avoidee.position;
            p.y += eyeHeight;
            return p;
        }

        void FaceTargetFlat(Vector3 worldTarget)
        {
            var pos = transform.position;
            var dir = new Vector3(worldTarget.x, pos.y, worldTarget.z) - pos;
            if (dir.sqrMagnitude > 0.001f)
                transform.rotation = Quaternion.LookRotation(dir.normalized, Vector3.up);
        }

        bool FindHiddenEscape(out Vector3 best)
        {
            lastSamples.Clear();
            lastHiddenFlags.Clear();
            best = default;

            float size = searchRadius * 2f;
            var sampler = new PoissonDiscSampler(size, size, poissonMinDistance, poissonAttemptsPerPoint);

            Vector3 origin = transform.position;
            Vector3 center = new(origin.x - size * 0.5f, origin.y, origin.z - size * 0.5f);
            Vector3 avoideeFlat = avoidee ? new Vector3(avoidee.position.x, origin.y, avoidee.position.z) : origin;

            bool foundAny = false;
            float bestScore = float.NegativeInfinity;

            foreach (var p in sampler.Samples())
            {
                var world = new Vector3(center.x + p.x, origin.y, center.z + p.y);

                if ((world - origin).sqrMagnitude > searchRadius * searchRadius) continue;
                if (!NavMesh.SamplePosition(world, out var hit, 1.5f, NavMesh.AllAreas)) continue;

                world = hit.position;

                bool hidden = IsHiddenFromAvoidee(world);

                lastSamples.Add(world);
                lastHiddenFlags.Add(hidden);
                if (!hidden) continue;

                float distFromAvoidee2 = (world - avoideeFlat).sqrMagnitude;
                float distFromUs2 = (world - origin).sqrMagnitude;

                // require a long move to avoid twitching near current position (?)
                if (distFromUs2 < (minMoveDistance * minMoveDistance)) continue;

                // prefer farther from avoidee, lightly penalize being very close to agent
                float score = distFromAvoidee2 - 0.25f * distFromUs2;

                if (score > bestScore)
                {
                    bestScore = score;
                    best = world;
                    foundAny = true;
                }
            }
            return foundAny;
        }

        bool IsHiddenFromAvoidee(Vector3 worldPoint)
        {
            if (useFOV)
            {
                Vector3 flatForward = avoidee.forward;
                flatForward.y = 0f;
                flatForward.Normalize();

                Vector3 toPoint = worldPoint - avoidee.position;
                toPoint.y = 0f;

                if (toPoint.sqrMagnitude > 0.0001f)
                {
                    float ang = Vector3.Angle(flatForward, toPoint);
                    if (ang > fovDegrees * 0.5f)
                        return true; // outside FOV is considered hidden
                }
            }

            // LOS ray test
            Vector3 start = AvoideePos();
            Vector3 end = worldPoint + Vector3.up * eyeHeight * 0.25f;
            Vector3 dir = end - start;
            float len = dir.magnitude;
            if (len <= 0.01f) return false;

            dir /= len;
            if (Physics.Raycast(start, dir, out RaycastHit hit, len, obstacleMask, QueryTriggerInteraction.Ignore))
                return true; // blocked -> hidden

            return false; // clear line -> visible
        }

        void OnDrawGizmosSelected()
        {
            if (!showGizmos) return;

            // rings
            Gizmos.color = new Color(1f, 0.3f, 0.2f, 0.8f);
            DrawWireDisc(transform.position, Vector3.up, dangerRadius);

            Gizmos.color = new Color(0.2f, 0.6f, 1f, 0.5f);
            DrawWireDisc(transform.position, Vector3.up, searchRadius);

            // chosen destination
            if (lastChosen.HasValue)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawSphere(lastChosen.Value + Vector3.up * 0.05f, 0.25f);
                Gizmos.DrawLine(transform.position, lastChosen.Value);
            }

            // sample points
            for (int i = 0; i < lastSamples.Count; i++)
            {
                var pt = lastSamples[i];
                bool hidden = lastHiddenFlags[i];

                Gizmos.color = hidden ? new Color(0.25f, 1f, 0.4f, 0.9f) : new Color(1f, 0.3f, 0.3f, 0.6f);
                Gizmos.DrawSphere(pt + Vector3.up * 0.05f, 0.12f);

                if (showSampleRays && avoidee)
                {
                    Gizmos.DrawLine(AvoideePos(), pt + Vector3.up * 0.25f);
                }
            }

            // avoidee FOV preview
            if (avoidee && useFOV)
            {
                Vector3 pos = avoidee.position + Vector3.up * 0.05f;
                float half = fovDegrees * 0.5f;
                Quaternion left = Quaternion.AngleAxis(-half, Vector3.up);
                Quaternion right = Quaternion.AngleAxis(half, Vector3.up);
                Vector3 fwd = Vector3.ProjectOnPlane(avoidee.forward, Vector3.up).normalized;
                Gizmos.color = new Color(1f, 1f, 0f, 0.5f);
                Gizmos.DrawLine(pos, pos + left * fwd * dangerRadius * 1.25f);
                Gizmos.DrawLine(pos, pos + right * fwd * dangerRadius * 1.25f);
            }

            // radial spokes + axes
            if (showRadialSpokes && spokes > 0 && (spokesRadius > 0f || searchRadius > 0f))
            {
                Vector3 origin = transform.position + Vector3.up * eyeHeight;
                float radius = (spokesRadius > 0f ? spokesRadius : searchRadius);
                float step = 360f / Mathf.Max(1, spokes);

                // axes (for orientation like the screenshot)
                if (showLocalAxes)
                {
                    Gizmos.color = axisZColor; // forward
                    Gizmos.DrawLine(origin, origin + transform.forward * axisLen);
                    Gizmos.color = axisXColor; // right
                    Gizmos.DrawLine(origin, origin + transform.right * axisLen);
                }

                for (int i = 0; i < spokes; i++)
                {
                    float ang = i * step;
                    // local XZ unit circle -> world dir, so the “flower” rotates with the avoider
                    Vector3 localDir = new Vector3(Mathf.Cos(ang * Mathf.Deg2Rad), 0f, Mathf.Sin(ang * Mathf.Deg2Rad));
                    Vector3 dir = transform.TransformDirection(localDir);

                    if (Physics.Raycast(origin, dir, out RaycastHit hit, radius, obstacleMask, QueryTriggerInteraction.Ignore))
                    {
                        Gizmos.color = spokeHit;
                        Vector3 end = stopSpokeAtHit ? hit.point : origin + dir * radius;
                        Gizmos.DrawLine(origin, end);
                    }
                    else
                    {
                        Gizmos.color = spokeClear;
                        Gizmos.DrawLine(origin, origin + dir * radius);
                    }
                }
            }
        }

        static void DrawWireDisc(Vector3 center, Vector3 normal, float radius, int segments = 48)
        {
            normal.Normalize();
            Vector3 any = Mathf.Abs(Vector3.Dot(normal, Vector3.up)) > 0.9f ? Vector3.right : Vector3.up;
            Vector3 tangent = Vector3.Cross(normal, any).normalized;
            Vector3 bitangent = Vector3.Cross(normal, tangent);

            Vector3 prev = center + tangent * radius;
            for (int i = 1; i <= segments; i++)
            {
                float t = (i / (float)segments) * Mathf.PI * 2f;
                Vector3 next = center + (tangent * Mathf.Cos(t) + bitangent * Mathf.Sin(t)) * radius;
                Gizmos.DrawLine(prev, next);
                prev = next;
            }
        }
    }
}
