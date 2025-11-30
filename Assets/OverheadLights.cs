using System.Collections.Generic;
using UnityEngine;

public class OverheadLights : MonoBehaviour
{
    public List<Light> lights;
    public Color currentColor;
    void Start()
    {
        lights = new List<Light>(GetComponentsInChildren<Light>());
        if (!GameManager.gameStarted)
        {
            currentColor = Color.red;
        }
    }

    // Update is called once per frame
    void Update()
    {
        foreach (Light light in lights)
        {
            light.color = currentColor;
        }
        if (GameManager.gameStarted)
        {
            currentColor = Color.green;
        }
    }
}
