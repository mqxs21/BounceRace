using UnityEngine;

public class Checkpoint : MonoBehaviour
{
    public int checkpointIndex;
    public bool isFinish = false;
    void Start()
    {
        if (isFinish)
        {
            checkpointIndex = 1;
        }
    }
    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Player")
        {
            other.gameObject.GetComponent<LapProgress>().CheckPointReached(checkpointIndex, isFinish);
        }
    }
    public int GetCheckpointIndex()
    {
        return checkpointIndex;
    }
}
