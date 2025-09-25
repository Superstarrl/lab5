using UnityEngine;
using UnityEngine.AI;

public class EnemyController : MonoBehaviour
{

    NavMeshAgent agent;

    private void Awake()
    {
        GetNMA();
    }

    private void Update()
    {
        
    }

    void GetNMA()
    {
        

        if (gameObject.GetComponent<NavMeshAgent>() == null)
        {
            Debug.LogWarning("Enemy Does not have a NavMeshAgent");
        }

        agent = gameObject.GetComponent<NavMeshAgent>();
    }





}
