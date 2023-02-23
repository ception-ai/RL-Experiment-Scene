using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RealtiveCameraToMap : MonoBehaviour
{
    private int sizeX , sizeZ;
    public GameObject realtiveObject;
    public Terrain terrain;

    void Start()
    {
        sizeX = (int)terrain.terrainData.size.x;
        sizeZ = (int)terrain.terrainData.size.z;
    }

    void Update()
    {
        transform.position = realtiveObject.transform.position;
        transform.position += new Vector3(sizeX /2 , sizeX + 10, sizeZ /2);
        transform.rotation = Quaternion.Euler(90.0f, 0.0f, 0.0f);
    }

}


